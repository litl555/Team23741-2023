package org.firstinspires.ftc.teamcode.FTC.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;

public class DriveSubsystem extends SubsystemBase {
    public CustomLocalization l;
    LoggerTool telemetry;

    public DriveSubsystem(CustomLocalization l, LoggerTool telemetry) {
        this.telemetry = telemetry;
        this.l = l;
    }

    // note that this is blocking and quite slow
    // this is only meant to run once or twice, not in a loop
    public double queryOuttakeObstacle() {
        double left = Robot.outtakeDistLeft.getDistance(DistanceUnit.MM);
        double right = Robot.outtakeDistRight.getDistance(DistanceUnit.MM);

        return (left + right) / 2.0;
    }
}
