package org.firstinspires.ftc.teamcode.FTC.Threading;

public class MathThread implements Runnable {
    public static boolean isRunning = false;
    @Override
    public void run() {
        synchronized (this) {
            isRunning = true;
        }

        synchronized (this) {
            isRunning = false;
        }
    }
}
