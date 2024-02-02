package org.firstinspires.ftc.teamcode.FTC.Threading;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.analysis.function.Log;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerData;
import org.firstinspires.ftc.teamcode.FTC.Localization.OdometryModule;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.TrajectoryRunner;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

import java.lang.reflect.Array;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.atomic.AtomicBoolean;

public class HardwareThread implements Runnable {
    private List<LynxModule> lynxModules;

    // odometry is stored in Robot.___pod, other values are stored here
    public double lastLiftPosition = 0, intakePower = 0, lastIntakeDist = -1;

    private final Deque<Double> liftQueue, wristQueue, armQueue, intakeQueue, droptakeQueue;
    private final Deque<Double[]> drivetrainQueue;
    private final Deque<ClawSubsystem.ClawState> clawQueue;
    public AtomicBoolean isRunning = new AtomicBoolean();

    private final ArrayList<Double> hardawareFps = new ArrayList<>();
    private final double avgFpsLength = 100;
    private long runCount = 0;

    public long timeAtHardwareReadStart = 0, timeAtHardwareReadEnd = 0;

    public final ThreadErrorDetection errorHandler;

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

        errorHandler = new ThreadErrorDetection("Hardware");
    }

    private long lastOdoRun = 0, lastOdoRunCountClear = 0;
    private int odoRunCount = 0;

    @Override
    public synchronized void run() { // TODO: gain stability at the cost of speed by making this sync
        try {
            if (isRunning.get() || errorHandler.catastrophicError.get()) return;
            isRunning.set(true);
            runCount++;

            long startTime = System.currentTimeMillis();
            Robot.telemetry.addImportant(new LoggerData("Hardware", startTime, "THREAD UPDATE"));

            //applyQueues();
            timeAtHardwareReadStart = System.currentTimeMillis();

            // read port values
            synchronized (Robot.class) {
                synchronized (Constants.class) {
                    for (LynxModule module : lynxModules) module.clearBulkCache();
                    // we have a problem where we will read significantly faster than we can write
                    // this only really matters for odometry, so force the robot to finish writing before we do more reading
                    long current = System.currentTimeMillis();

                    if (current - lastOdoRun > 7 && (current - lastOdoRun >= 20 || drivetrainQueue.size() < 3)) { // allow max 3 stale writes or at most 20 ms for writes
                        lastOdoRun = current;

                        Robot.leftPod.getDelta();
                        Robot.rightPod.getDelta();
                        Robot.backPod.getDelta();

                        Robot.customLocalization.update();

                        TrajectoryRunner tr = Robot.math.trajectoryRunner.get();
                        if (tr != null) {
                            // this is a scuffed system because this is essentially a queue system -> regular code calls tr.update which
                            // sends a request to update the trajectory runner
                            // what this means though is that this thread doesn't actually control the trajectory runner
                            // and actually the regular code (that called tr.update) controls the current state of the trajectory runner
                            // (specifically switching FINISHED to PRESTART), so filter those. we only want to run "fresh" trajectories
                            if (tr.currentState == TrajectoryRunner.State.FINISHED || tr.currentState == TrajectoryRunner.State.PRESTART) {
                                Robot.math.trajectoryRunner.set(null);
                            } else {
                                tr._update();
                            }
                        }

                        odoRunCount++;
                    }

                    lastLiftPosition = Robot.liftEncoder.getCurrentPosition();

                    Robot.liftSubsystem.calculateControllerPower();
                }
            }

            timeAtHardwareReadEnd = System.currentTimeMillis();

            //lastLiftPosition = Robot.liftEncoder.getCurrentPosition();
            intakePower = Robot.intakeMotor.getPower();

            // note that we cannot read and write values at the same time (at least i dont think so)
            // so there is not point having read/write in different threads
            // instead just call write at every possible opportunity lol
            //applyQueues();

            // if were not currently do calculations queue them up
            //if (!Robot.math.isRunning.get()) Robot.mathThread.start();

            if (Robot.intakeSubsystem.activateIntakeDist.get()) lastIntakeDist = Robot.intakeDist.getDistance(DistanceUnit.MM);
            else lastIntakeDist = -1;

            //applyQueues();

            // timing stuff
            long endTime = System.currentTimeMillis();
            if (runCount % 20 == 0) {
                long delta = endTime - startTime;
                Robot.telemetry.addImportant(new LoggerData("Hardware", formatFps(delta, hardawareFps), "THREAD LENGTH"));
            }

            if (endTime - lastOdoRunCountClear > 500) {
                lastOdoRunCountClear = endTime;

                String len = truncate((int) (1000.0 / (Math.max(odoRunCount, 1) * 2.0)), 3);
                Robot.telemetry.addImportant(new LoggerData("Odometry Sub-Thread", len + " ms (" + (odoRunCount * 2) + " FPS)", "THREAD LENGTH"));

                odoRunCount = 0;
            }

            // occasionally (and a lot more freq. before this func was sync), we could get multiple instances of this thread running
            if (!isRunning.get()) errorHandler.registerError("Multiple instances of thread detected");

            isRunning.set(false);
        } catch (Exception e) { errorHandler.registerError(e); }
    }

    private double getAverageFps(ArrayList<Double> arr, Double fps) {
        // first value in arr is the current fps average
        Double a = fps / avgFpsLength;
        if (arr.size() != avgFpsLength) {
            arr.clear();
            arr.add(fps);
            for (int i = 1; i < avgFpsLength; i++) arr.add(a);

            return fps;
        } else {
            arr.set(0, arr.get(0) + a - arr.get(1));
            arr.remove(1);
            arr.add(a);

            return arr.get(0);
        }
    }

    private String formatFps(double delta, ArrayList<Double> arr) {
        delta = Math.max(1, delta);
        return truncate((int) delta, 3) + " ms (" + truncate((int) getAverageFps(arr, 1000.0 / delta), 4) + " FPS)";
    }

    private String truncate(int d, int n) { return (d + "").substring(0, Math.min(n, (d + "").length())); }

    public void applyQueues() {
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

    private void applyDroptake(double d) {
        Robot.droptakeRight.setPosition(1.0 - d);
        Robot.droptakeLeft.setPosition(d);
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

    private ArrayList<Double> hrs = new ArrayList<>();
    private ArrayList<Double> hre = new ArrayList<>();
    private ArrayList<Double> ots = new ArrayList<>();
    private ArrayList<Double> ome = new ArrayList<>();

    private void applyDrivetrain(Double[] d) {
        Robot.rightFront.setPower(d[0]);
        Robot.leftFront.setPower(d[1]);
        Robot.rightRear.setPower(d[2]);
        Robot.leftRear.setPower(d[3]);

        // TODO: this is slow (having to sync 4 times), only do this for debugging
        if (Robot.math.isUpdatingOdometry && Robot.isBusy) {
            long t = System.currentTimeMillis();

            Robot.telemetry.addImportant(new LoggerData("Hardware read start", formatFps(t - timeAtHardwareReadStart, hrs), "ODOMETRY TIMING"));
            Robot.telemetry.addImportant(new LoggerData("Hardware read end",  formatFps(t - timeAtHardwareReadEnd, hre), "ODOMETRY TIMING"));
            Robot.telemetry.addImportant(new LoggerData("Odo thread start", formatFps(t - Robot.math.timeAtThreadCalled, ots), "ODOMETRY TIMING"));
            Robot.telemetry.addImportant(new LoggerData("Odo math end", formatFps(t - Robot.math.timeAtCalculationFinished, ome), "ODOMETRY TIMING"));
        }
    }

    private <T> void apply(Deque<T> queue, ApplyQueueAction<T> action, String name) {
        // note: there is a race condition here
        // since .addImportant is synced, there is a considerable gap between calling this function
        // and calling queue.getLast(), which allows the queue to be updated in that time
        // note that this is despite us explicitly calling sync (queue) above
        if (queue.size() == 0) return;
        //if (queue.size() > 1) Robot.telemetry.addImportant("WARN",  name + " queue has backed up values");

        try {
            T o = queue.getLast();
            queue.clear();

            action.run(o);
        } catch (Exception e) {
            Robot.telemetry.addError("Write Queue", e);
        }
    }

    public void setLiftPower(double power) { liftQueue.addLast(power); }
    public void setDrivetrain(double fl, double fr, double bl, double br) { drivetrainQueue.addLast(new Double[] {fl, fr, bl, br}); }
    public void setWrist(double angle) { wristQueue.addLast(angle); }
    public void setIntakePower(double p) { intakeQueue.addLast(p); }
    public void setDroptake(double d) { droptakeQueue.addLast(d); }
    public void setArm(double angle) { armQueue.addLast(angle); }
    public void setClaw(ClawSubsystem.ClawState state) { clawQueue.addLast(state); }

    private interface ApplyQueueAction<T> {
        public void run(T o);
    }
}