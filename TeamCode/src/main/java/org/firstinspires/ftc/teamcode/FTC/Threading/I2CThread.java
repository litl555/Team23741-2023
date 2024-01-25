package org.firstinspires.ftc.teamcode.FTC.Threading;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

public class I2CThread implements Runnable {
    @Override
    public void run() {
        while (Robot.caller.opModeIsActive() && !Robot.caller.isStopRequested()) {
            Robot.telemetry.addImportant("dist from thread", Robot.intakeDist.getDistance(DistanceUnit.MM));
        }
    }
}
