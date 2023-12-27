package org.firstinspires.ftc.teamcode.FTC.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@TeleOp
public class Robotcentric2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        CustomLocalization l = new CustomLocalization(new Pose2d(000, 000, 0.0), hardwareMap);

        LoggerTool telemetry1 = new LoggerTool(telemetry);
        Robot.telemetry = telemetry1;
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            l.setWeightedDrivePowers(new Pose2d(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x));
            l.update();
            Robot.telemetry.add("angle", Constants.angle);
            Robot.telemetry.add("angle1", Constants.robotPose.getHeading());

            Robot.telemetry.update();
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();

            }
        }
    }
}
