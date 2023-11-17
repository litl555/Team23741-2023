package org.firstinspires.ftc.teamcode.FTC.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;

public class DriveSubsystem extends SubsystemBase {
    CustomLocalization l;

    public DriveSubsystem(CustomLocalization l) {
        this.l = l;
    }

    @Override
    public void periodic() {
        l.update();
    }
}
