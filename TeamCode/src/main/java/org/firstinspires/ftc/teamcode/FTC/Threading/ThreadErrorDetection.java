package org.firstinspires.ftc.teamcode.FTC.Threading;

import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerData;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

import java.util.concurrent.atomic.AtomicBoolean;

public class ThreadErrorDetection {
    private final int maxError = 5, maxTime = 1_000;

    private final String name;
    private final Thread thread;
    private int errorCount;
    private long lastClearTime = 0, timeSinceLastIntent = 0;
    private final long timeout;

    public AtomicBoolean catastrophicError = new AtomicBoolean(false);

    // java exectuorService offers a timeout functionality but uh im not switching apis again
    public ThreadErrorDetection(String name, Thread thread, long timeout) {
        this.name = name;
        this.thread = thread;
        lastClearTime = System.currentTimeMillis();
        timeSinceLastIntent = System.currentTimeMillis();
        this.timeout = timeout;
    }

    public void update(long timeAtCall) { // minor optimization if you've prev alr called currentTimeMillis
        if (catastrophicError.get()) return;

        // count how many errors we get every second
        if (timeAtCall - lastClearTime > maxTime) {
            errorCount = 0;
            lastClearTime = timeAtCall;
        }

        if (errorCount > maxError) {
            catastrophicError.set(true);
            Robot.telemetry.addImportant(new LoggerData(name + " Status", "CATASTROPHIC ERROR", "THREAD UPDATE"));

            if (thread.isAlive()) thread.interrupt();
        }

        // timeout logic
        if (timeAtCall - timeSinceLastIntent > timeout && thread.isAlive()) {
            registerError("Thread timed out");
            // recheck due to possibility that the thread could have ended while we were logging the data
            if (thread.isAlive()) thread.interrupt();
        }
    }

    public void update() { update(System.currentTimeMillis()); }

    // because its multithread were not guaranteed to be able to detect when a thread starts
    // so instead have the thread inform us
    public void notifyIntentToTryRun() { timeSinceLastIntent = System.currentTimeMillis(); }

    public void registerError(Exception e) {
        Robot.telemetry.addError(name, e);
        errorCount++;
    }

    public void registerError(String s) {
        Robot.telemetry.addError(name, new Exception(s));
        errorCount++;
    }
}
