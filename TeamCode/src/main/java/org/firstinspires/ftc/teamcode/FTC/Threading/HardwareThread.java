package org.firstinspires.ftc.teamcode.FTC.Threading;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.FTC.Localization.OdometryModule;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Queue;

// this will run on the main thread, so no need to thread this
public class HardwareThread implements Runnable {
    private List<LynxModule> lynxModules;

    // odometry is stored in Robot.___pod, other values are stored here
    public double lastLiftPosition = 0;

    private final Deque<Double> liftQueue, wristQueue, armQueue;
    private final Deque<Double[]> drivetrainQueue;
    private final Deque<ClawSubsystem.ClawState> clawQueue;
    private boolean isRunning = false, hasErroredOut = false;

    public HardwareThread(HardwareMap map) {
        lynxModules = map.getAll(LynxModule.class);

        for (LynxModule module : lynxModules) module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        liftQueue = new ArrayDeque<>();
        wristQueue = new ArrayDeque<>();
        armQueue = new ArrayDeque<>();
        drivetrainQueue = new ArrayDeque<>();
        clawQueue = new ArrayDeque<>();
    }

    @Override
    public void run() {
        try {
            if (isRunning || hasErroredOut) return;
            isRunning = true;

            long startTime = System.currentTimeMillis();
            Robot.telemetry.addImportant("Hardware Thread Last Update", startTime);

            applyQueues();

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

            lastLiftPosition = Robot.liftEncoder.getCurrentPosition();
            // TODO: read sensor values via photonFTC

            // note that we cannot read and write values at the same time (at least i dont think so)
            // so there is not point having read/write in different threads
            // instead just call write at every possible opportunity lol
            applyQueues();


            // TODO
            // check if robot is currently running calculations, if not start them
            // write to queues

            long endTime = System.currentTimeMillis();
            long delta = endTime - startTime;
            Robot.telemetry.addImportant("Hardware Thread",
                truncate((int) delta, 3) + " ms (" + truncate((int) (1000.0 / delta), 3) + " FPS)");

            isRunning = false;
        } catch (Exception e) {
            // TODO: remove/get better error handling, this is only for testing
            Robot.telemetry.addImportant("ERROR", e);
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

        //leftRear.setPower(bl);
        //leftFront.setPower(fl);
        //rightFront.setPower(fr);
        //rightRear.setPower(br);
    }

    private <T> void apply(Deque<T> queue, ApplyQueueAction<T> action, String name) {
        if (queue.size() == 0) return;
        if (queue.size() > 1) Robot.telemetry.addImportant("WARN",  name + " queue has backed up values");

        T o = queue.getLast();
        queue.clear();

        action.run(o);
    }

    public void setLiftPower(double power) { liftQueue.addLast(power); }

    public void setDrivetrain(double fl, double fr, double bl, double br) { drivetrainQueue.addLast(new Double[] {fl, fr, bl, br}); }

    public void setWrist(double angle) { wristQueue.addLast(angle); }

    public void setArm(double angle) { armQueue.addLast(angle); }

    public void setClaw(ClawSubsystem.ClawState state) { clawQueue.addLast(state); }

    private interface ApplyQueueAction<T> {
        public void run(T o);
    }
}