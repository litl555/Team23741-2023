package org.firstinspires.ftc.teamcode.FTC.Threading;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerData;
import org.firstinspires.ftc.teamcode.FTC.Localization.OdometryModule;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.TrajectoryRunner;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.atomic.AtomicBoolean;

public class HardwareThread implements Runnable {
    private List<LynxModule> lynxModules;

    // odometry is stored in Robot.___pod, other values are stored here
    public double lastLiftPosition = 0, intakePower = 0;

    private final Deque<Double> liftQueue, wristQueue, armQueue, intakeQueue;
    private final Deque<Double[]> drivetrainQueue;
    private final Deque<ClawSubsystem.ClawState> clawQueue;
    private final Deque<IntakeSubsystem.IntakePosition> droptakeQueue;
    private boolean hasErroredOut = false;
    public AtomicBoolean isRunning = new AtomicBoolean();

    private final ArrayList<Double> avgFps = new ArrayList<>();
    private final double avgFpsLength = 100;
    private double avgFpsValue = 0;
    private long runCount = 0;

    public long timeAtHardwareReadStart = 0, timeAtHardwareReadEnd = 0;

    public Exception ex = null; // TODO: add better error logging

    public HardwareThread(HardwareMap map) {
        lynxModules = map.getAll(LynxModule.class);

        for (LynxModule module : lynxModules) module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        liftQueue = new ArrayDeque<>();
        wristQueue = new ArrayDeque<>();
        armQueue = new ArrayDeque<>();
        drivetrainQueue = new ArrayDeque<>();
        clawQueue = new ArrayDeque<>();
        intakeQueue = new ArrayDeque<>();
        droptakeQueue = new ArrayDeque<>();
    }

    @Override
    public void run() {
        try {
            if (isRunning.get() || hasErroredOut) return;
            isRunning.set(true);
            runCount++;

            long startTime = System.currentTimeMillis();
            Robot.telemetry.addImportant(new LoggerData("Hardware", startTime, "THREAD UPDATE"));

            applyQueues();
            timeAtHardwareReadStart = System.currentTimeMillis();

            // read port values
            for (LynxModule module : lynxModules) module.clearBulkCache();
            synchronized (Robot.leftPod) {
                synchronized (Robot.rightPod) {
                    synchronized (Robot.backPod) {
                        Robot.leftPod.getDelta();
                        Robot.rightPod.getDelta();
                        Robot.backPod.getDelta();
                    }
                }
            }

            timeAtHardwareReadEnd = System.currentTimeMillis();

            lastLiftPosition = Robot.liftEncoder.getCurrentPosition();
            intakePower = Robot.intakeMotor.getPower();

            // TODO: read sensor values via photonFTC

            // note that we cannot read and write values at the same time (at least i dont think so)
            // so there is not point having read/write in different threads
            // instead just call write at every possible opportunity lol
            applyQueues();

            // if were not currently do calculations queue them up
            if (!Robot.math.isRunning.get()) Robot.mathThread.start();

            applyQueues();

            // timing stuff
            long endTime = System.currentTimeMillis();
            long delta = endTime - startTime;

            if (runCount % 20 == 0) {
                double fps = (1000.0 / delta) / avgFpsLength;

                // saturate at start
                if (avgFps.size() == 0) {
                    for (int i = 0; i < avgFpsLength; i++) avgFps.add(fps);
                    avgFpsValue = (1000.0 / delta);
                }

                avgFpsValue += fps;
                if (avgFps.size() >= avgFpsLength) {
                    avgFpsValue -= avgFps.get(0);
                    avgFps.remove(0);
                }
                avgFps.add(fps);
            }

            Robot.telemetry.addImportant(new LoggerData("Hardware",
                truncate((int) delta, 3) + " ms (" + truncate((int) avgFpsValue, 3) + " Avg FPS)", "THREAD LENGTH"));

            if (!isRunning.get()) {
                Robot.telemetry.addImportant("BIG ERROR", "detected multiple hardware threads running");
            }

            isRunning.set(false);
        } catch (Exception e) {
            // TODO: remove/get better error handling, this is only for testing
            ex = e;
            Robot.telemetry.addImportant("HARDWARE ERROR", e);
            Robot.telemetry.addImportant("HARDWARE ERROR STACKTRACE 0", e.getStackTrace()[0]);
            Robot.telemetry.addImportant("HARDWARE ERROR STACKTRACE 1", e.getStackTrace()[1]);
            hasErroredOut = true;
        }
    }

    private String truncate(int d, int n) { return (d + "").substring(0, Math.min(n, (d + "").length())); }

    private void applyQueues() {
        synchronized (liftQueue) { apply(liftQueue, this::applyLift, "Lift"); }
        synchronized (wristQueue) { apply(wristQueue, this::applyWrist, "Wrist"); }
        synchronized (armQueue) { apply(armQueue, this::applyArm, "Arm"); }
        synchronized (clawQueue) { apply(clawQueue, this::applyClaw, "Claw"); }
        synchronized (drivetrainQueue) { apply(drivetrainQueue, this::applyDrivetrain, "Drivetrain"); }
        synchronized (intakeQueue) { apply(intakeQueue, this::applyIntake, "Intake"); }
        synchronized (droptakeQueue) { apply(droptakeQueue, this::applyDroptake, "Droptake"); }
    }

    private void applyLift(Double d) {
        Robot.liftLeft.setPower(-d);
        Robot.liftRight.setPower(d);
    }

    private void applyWrist(Double d) {
        Robot.wristRed.setPosition(d);
        Robot.wristBlue.setPosition(d);
    }

    private void applyArm(Double d) {
        Robot.armYellow.setPosition(d);
        Robot.armGreen.setPosition(d);
    }

    private void applyIntake(Double d) {
        Robot.intakeMotor.setPower(d);
        Robot.bottomRoller.setPower(d);
    }

    private void applyDroptake(IntakeSubsystem.IntakePosition p) {
        switch (p) {
            case UP:
                Robot.droptakeRight.setPosition(1.0 - IntakeSubsystem.intakeUpPosition);
                Robot.droptakeLeft.setPosition(IntakeSubsystem.intakeUpPosition);
                break;
            case DOWN:
                Robot.droptakeRight.setPosition(1.0 - IntakeSubsystem.intakeDownPosition);
                Robot.droptakeLeft.setPosition(IntakeSubsystem.intakeDownPosition);
                break;
        }
    }

    private void applyClaw(ClawSubsystem.ClawState s) {
        switch (s) {
            case OPEN:
                Robot.clawBlack.setPosition(ClawSubsystem.openPos);
                Robot.clawWhite.setPosition(ClawSubsystem.openPos);
                break;
            case CLOSED:
                Robot.clawWhite.setPosition(ClawSubsystem.closedPosTop);
                Robot.clawBlack.setPosition(ClawSubsystem.closedPosBot);
                break;
            case OPENONE:
                Robot.clawBlack.setPosition(ClawSubsystem.openPos);
                Robot.clawWhite.setPosition(ClawSubsystem.closedPosBot);
                break;
            case HALFCLOSE:
                Robot.clawWhite.setPosition(ClawSubsystem.halfPos);
                Robot.clawBlack.setPosition(ClawSubsystem.halfPos);
                break;
        }
    }

    private void applyDrivetrain(Double[] d) {
        Robot.rightFront.setPower(d[0]);
        Robot.leftFront.setPower(d[1]);
        Robot.rightRear.setPower(d[2]);
        Robot.leftRear.setPower(d[3]);
        
        if (Robot.math.isUpdatingOdometry && Robot.isBusy) {
            long t = System.currentTimeMillis();
            Robot.telemetry.addImportant(new LoggerData("Hardware read start", t - timeAtHardwareReadStart, "ODOMETRY TIMING"));
            Robot.telemetry.addImportant(new LoggerData("Hardware read end", t - timeAtHardwareReadEnd, "ODOMETRY TIMING"));
            Robot.telemetry.addImportant(new LoggerData("Odo thread start", t - Robot.math.timeAtThreadCalled, "ODOMETRY TIMING"));
            Robot.telemetry.addImportant(new LoggerData("Odo math end", t - Robot.math.timeAtCalculationFinished, "ODOMETRY TIMING"));
        }
    }

    private <T> void apply(Deque<T> queue, ApplyQueueAction<T> action, String name) {
        // note: there is a race condition here
        // since .addImportant is synced, there is a considerable gap between calling this function
        // and calling queue.getLast(), which allows the queue to be updated in that time
        // note that this is despite us explicitly calling sync (queue) above
        if (queue.size() == 0) return;
        //if (queue.size() > 1) Robot.telemetry.addImportant("WARN",  name + " queue has backed up values");

        T o = queue.getLast();
        queue.clear();

        action.run(o);
    }

    public void setLiftPower(double power) { liftQueue.addLast(power); }
    public void setDrivetrain(double fl, double fr, double bl, double br) { drivetrainQueue.addLast(new Double[] {fl, fr, bl, br}); }
    public void setWrist(double angle) { wristQueue.addLast(angle); }
    public void setIntakePower(double p) { intakeQueue.addLast(p); }
    public void setDroptake(IntakeSubsystem.IntakePosition p) { droptakeQueue.addLast(p); }
    public void setArm(double angle) { armQueue.addLast(angle); }
    public void setClaw(ClawSubsystem.ClawState state) { clawQueue.addLast(state); }

    private interface ApplyQueueAction<T> {
        public void run(T o);
    }
}