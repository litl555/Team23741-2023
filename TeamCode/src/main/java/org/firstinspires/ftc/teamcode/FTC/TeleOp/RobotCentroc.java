package org.firstinspires.ftc.teamcode.FTC.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;

@Config
@TeleOp

public class RobotCentroc extends LinearOpMode {
    public static double kp = -2;
    public static double desAngle = 0.0;
    public static double speed = 4;

    @Override
    public void runOpMode() throws InterruptedException {
        LoggerTool telemetry = new LoggerTool();
        CustomLocalization l = new CustomLocalization(new Pose2d(0, 0, 0), hardwareMap);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            desAngle -= Math.toRadians(gamepad1.right_stick_x * speed);
            telemetry.add("desAngle", desAngle);
            telemetry.add("Angle", Constants.angle);
            telemetry.add("x", l.getPoseEstimate().getX());
            telemetry.add("y", l.getPoseEstimate().getY());
            telemetry.update();
            l.setWeightedDrivePowers(new Pose2d(Math.pow(gamepad1.left_stick_x, 2) * Math.signum(gamepad1.left_stick_x), Math.pow(gamepad1.left_stick_y, 2) * Math.signum(gamepad1.left_stick_y), kp * (desAngle - Constants.angle)));
            l.update();
        }
    }
}
