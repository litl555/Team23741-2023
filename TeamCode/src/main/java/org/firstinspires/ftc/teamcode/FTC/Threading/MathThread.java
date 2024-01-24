package org.firstinspires.ftc.teamcode.FTC.Threading;

import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.TrajectoryRunner;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

// this is called by HardwareThread, not Robot
// bit convoluted but its the easiest way to allow odo to run as soon as possible after ports are read
// since there is no point in running odo multiple times over the exact same values
// in practice were basically moving whatever the periodic functions are into this thread
public class MathThread implements Runnable {
    private boolean hasErroredOut = false;
    public final AtomicBoolean isRunning = new AtomicBoolean();
    public final AtomicReference<TrajectoryRunner> trajectoryRunner = new AtomicReference<>();

    public MathThread() {
        trajectoryRunner.set(null);
        isRunning.set(false);
    }

    @Override
    public void run() {
        try {
            if (isRunning.get() || hasErroredOut) return;
            isRunning.set(true);

            long startTime = System.currentTimeMillis();
            Robot.telemetry.addImportant("Math Thread Last Update", startTime);

            Robot.liftSubsystem.calculateControllerPower();

            synchronized (Constants.class) { Robot.customLocalization.update(); }

            // then check if there are any trajectories we need to run
            TrajectoryRunner tr = trajectoryRunner.get();
            if (tr != null) {
                // this is a scuffed system because this is essentially a queue system -> regular code calls tr.update which
                // sends a request to well, update the trajectory runner
                // what this means though is that this thread doesn't actually control the trajectory runner
                // and actually the regular code (that called tr.update) controls the current state of the trajectory runner
                // (specifically switching FINISHED to PRESTART), so filter those. we only want to run "fresh" trajectories
                if (tr.currentState == TrajectoryRunner.State.FINISHED || tr.currentState == TrajectoryRunner.State.PRESTART) {
                    trajectoryRunner.set(null);
                } else tr._update();
            }

            long endTime = System.currentTimeMillis();
            long delta = Math.max(endTime - startTime, 1);
            Robot.telemetry.addImportant("Math Thread",
                truncate((int) delta, 3) + " ms");

            isRunning.set(false);
        } catch (Exception e) {
            // TODO: remove/get better error handling, this is only for testing
            Robot.telemetry.addImportant("MATH ERROR", e);
            hasErroredOut = true;
        }
    }

    // TODO we need to make a helper class eventually lol
    private String truncate(int d, int n) { return (d + "").substring(0, Math.min(n, (d + "").length())); }
}
