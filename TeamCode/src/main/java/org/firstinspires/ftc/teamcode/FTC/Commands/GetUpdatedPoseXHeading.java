package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

public class GetUpdatedPoseXHeading extends InstantCommand {

    public GetUpdatedPoseXHeading(CustomLocalization l) {
        super(
                () -> {
                    double backDropPos = 1500;
                    double angle = Math.atan2(Robot.distance2.getDistance(DistanceUnit.MM) - Robot.distance1.getDistance(DistanceUnit.MM), Robot.distanceBetween);
                    double x = Robot.distance1.getDistance(DistanceUnit.MM) * Math.sin(angle);
                    Constants.robotPose = new Pose2d(backDropPos - x, Constants.robotPose.getY(), angle);
                    Constants.angle = angle;
                    l.pose = new Pose2d(backDropPos - x, l.pose.getY(), angle);
                }
        );
    }

}
