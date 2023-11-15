package org.firstinspires.ftc.teamcode.FTC.PathFollowing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;

@Autonomous
public class LineRunnerTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Line line = new Line(new Pose2d(0, 0), new Pose2d(1, 1000));
        CustomLocalization l = new CustomLocalization(new Pose2d(0, 0, 0), hardwareMap);
        LineRunner lr = new LineRunner(hardwareMap, l, line, 0, LineRunner.HeadingType.ConstantHeadingVelo);
        LoggerTool telemetry = new LoggerTool();
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
