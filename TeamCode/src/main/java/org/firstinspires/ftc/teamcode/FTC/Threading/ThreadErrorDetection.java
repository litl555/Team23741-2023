package org.firstinspires.ftc.teamcode.FTC.Threading;

import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerData;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

import java.util.concurrent.atomic.AtomicBoolean;

public class ThreadErrorDetection {
    private final int maxError = 5, maxTime = 1_000;

    private final String name;
    private Thread thread;
    private int errorCount;
    private long lastClearTime = 0;

    public AtomicBoolean catastrophicError = new AtomicBoolean(false);

    // java exectuorService offers a timeout functionality but uh im not switching apis again
    public ThreadErrorDetection(String name) {
        this.name = name;
        lastClearTime = System.currentTimeMillis();
    }

    public void assignThread(Thread t) {
        this.thread = t;
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
    }

    public void update() { update(System.currentTimeMillis()); }


    public void registerError(Exception e) {
        Robot.telemetry.addError(name, e);
        errorCount++;
    }

    public void registerError(String s) {
        Robot.telemetry.addError(name, new Exception(s));
        errorCount++;
    }
}
