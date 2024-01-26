package org.firstinspires.ftc.teamcode.FTC.Threading;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerData;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

public class WriteThread implements Runnable {
    private LinearOpMode watchdog;

    public WriteThread(LinearOpMode watchdog) {
        this.watchdog = watchdog;
    }

    @Override
    public void run() {
        while (watchdog.opModeIsActive() && !watchdog.isStopRequested()) {
            long startTime = System.currentTimeMillis();
            Robot.telemetry.addImportant(new LoggerData("Write", startTime, "THREAD UPDATE"));

            Robot.hardware.applyQueues();

            Robot.telemetry.addImportant(new LoggerData("Write", (System.currentTimeMillis() - startTime) + " ms", "THREAD LENGTH"));
        }
    }
}
