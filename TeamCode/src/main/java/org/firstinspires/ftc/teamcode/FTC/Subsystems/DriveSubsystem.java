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
}
