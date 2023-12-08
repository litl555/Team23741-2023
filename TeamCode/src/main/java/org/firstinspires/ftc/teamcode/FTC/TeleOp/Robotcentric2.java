package org.firstinspires.ftc.teamcode.FTC.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class Robotcentric2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        CustomLocalization l = new CustomLocalization(new Pose2d(300, -1500, -Math.PI / 2.0), hardwareMap);
        LoggerTool telemetry = Robot.telemetry;
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            l.setWeightedDrivePowers(new Pose2d(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x));
            l.update();
            telemetry.add("angle", Constants.angle);

            telemetry.update();
        }
    }
}
