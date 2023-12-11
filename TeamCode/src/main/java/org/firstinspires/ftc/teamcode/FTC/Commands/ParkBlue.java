package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

public class ParkBlue extends SequentialCommandGroup {
    double startAngle = 0.0;

    public ParkBlue() {
        startAngle = -Constants.angle;
        addCommands(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> Robot.l.setWeightedDrivePowers(new Pose2d(0.0, -.3, 0))),
                                new WaitCommand(200),
                                new InstantCommand(() -> Robot.l.setWeightedDrivePowers(new Pose2d(.3, 0.0, 0))),
                                new WaitCommand(4000),
                                new InstantCommand(() -> Robot.l.setWeightedDrivePowers(new Pose2d(0.0, .3, 0.0))),
                                new WaitCommand(2000),
                                new InstantCommand(() -> Robot.l.setWeightedDrivePowers(new Pose2d(0.0, 0.0, 0.0)))

                        ),
                        new SequentialCommandGroup(
                                new GoToHeight(Robot.lift, Robot.claw, 1),
                                new WaitCommand(200),
                                new GoToHeight(Robot.lift, Robot.claw, 0)
                        )


                )

        );
    }
}
