package org.firstinspires.ftc.teamcode.FTC.Threading;

import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerData;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.TrajectoryRunner;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

// this is called by HardwareThread, not Robot
// bit convoluted but its the easiest way to allow odo to run as soon as possible after ports are read
// since there is no point in running odo multiple times over the exact same values
// in practice were basically moving whatever the periodic functions are into this thread
public class MathThread implements Runnable {
    private boolean hasErroredOut = false;
    public final AtomicBoolean isRunning = new AtomicBoolean();
    public final AtomicReference<TrajectoryRunner> trajectoryRunner = new AtomicReference<>();
    public boolean isUpdatingOdometry = false;
    public long timeAtCalculationFinished = 0, timeAtThreadCalled = 0;

    public ThreadErrorDetection errorHandler;

    public MathThread() {
        trajectoryRunner.set(null);
        isRunning.set(false);

        errorHandler = new ThreadErrorDetection("Math", Robot.mathThread, 40);
    }

    @Override
    public void run() {
        errorHandler.notifyIntentToTryRun();

        try {
            if (isRunning.get() || errorHandler.catastrophicError.get()) return;
            isRunning.set(true);

            long startTime = System.currentTimeMillis();
            Robot.telemetry.addImportant(new LoggerData("Math", startTime, "THREAD UPDATE"));
            timeAtThreadCalled = startTime;

            Robot.liftSubsystem.calculateControllerPower();

            synchronized (Constants.class) { Robot.customLocalization.update(); }

            // then check if there are any trajectories we need to run
            TrajectoryRunner tr = trajectoryRunner.get();
            if (tr != null) {
                // this is a scuffed system because this is essentially a queue system -> regular code calls tr.update which
                // sends a request to update the trajectory runner
                // what this means though is that this thread doesn't actually control the trajectory runner
                // and actually the regular code (that called tr.update) controls the current state of the trajectory runner
                // (specifically switching FINISHED to PRESTART), so filter those. we only want to run "fresh" trajectories
                if (tr.currentState == TrajectoryRunner.State.FINISHED || tr.currentState == TrajectoryRunner.State.PRESTART) {
                    trajectoryRunner.set(null);
                } else {
                    tr._update();
                    isUpdatingOdometry = true;
                    timeAtCalculationFinished = System.currentTimeMillis();
                }
            } else isUpdatingOdometry = false;

            long endTime = System.currentTimeMillis();
            long delta = Math.max(endTime - startTime, 1);
            Robot.telemetry.addImportant(new LoggerData("Math", truncate((int) delta, 3) + " ms", "THREAD LENGTH"));

            isRunning.set(false);
        } catch (Exception e) { errorHandler.registerError(e); }
    }

    // TODO we need to make a helper class eventually lol
    private String truncate(int d, int n) { return (d + "").substring(0, Math.min(n, (d + "").length())); }
}
