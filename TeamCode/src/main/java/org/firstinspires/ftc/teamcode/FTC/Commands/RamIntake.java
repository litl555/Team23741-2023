package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

@Config
public class RamIntake extends SequentialCommandGroup {
    public static double ramPowerMag = .2;
    public static int ramTime = 700;

    public RamIntake() {
        addCommands(
            new InstantCommand(() -> Robot.customLocalization.setWeightedDrivePowers(new Pose2d(0.0, -ramPowerMag, 0))),
            new WaitCommand(ramTime),
            new InstantCommand(() -> Robot.customLocalization.setWeightedDrivePowers(new Pose2d(0, 0, 0))));
    }
}
