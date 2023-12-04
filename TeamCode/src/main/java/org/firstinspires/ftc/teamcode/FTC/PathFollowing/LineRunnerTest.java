package org.firstinspires.ftc.teamcode.FTC.PathFollowing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class LineRunnerTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive dr = new SampleMecanumDrive(hardwareMap);

        Line line = new Line(new Pose2d(0, 0), new Pose2d(1, 1000), true, true);
        CustomLocalization l = new CustomLocalization(new Pose2d(0, 0, 0), hardwareMap, dr);
        LineRunner lr = new LineRunner(hardwareMap, l, line, 0, LineRunner.HeadingType.ConstantHeadingVelo);
        LoggerTool telemetry = Robot.telemetry;
        waitForStart();

        lr.start();
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.add("equation", lr.t.equation(lr.t.velosSpaced.get(lr.ind)));
            telemetry.update();
            lr.update();
            l.update();
        }
    }
}
