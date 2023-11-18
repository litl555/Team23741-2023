package org.firstinspires.ftc.teamcode.FTC.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.Line;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.LineRunner;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.Trajectory;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.TrajectoryRunner;

@TeleOp
public class LineTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        LoggerTool telemetry = new LoggerTool();
        CustomLocalization l = new CustomLocalization(new Pose2d(0, 0, 0), hardwareMap);
//        Trajectory trajectory = new Trajectory(new Pose2d(0, 0), new Pose2d(0, 500), new Pose2d(-10, 0), new Pose2d(10, 0), new Pose2d(0, 0), new Pose2d(0, 0), true, true);
        Line line = new Line(new Pose2d(0, 0, 0), new Pose2d(500, 0, 0), true, true);
//        TrajectoryRunner tr = new TrajectoryRunner(hardwareMap, l, trajectory, 0, TrajectoryRunner.HeadingType.ConstantHeadingVelo, telemetry);
        LineRunner tr = new LineRunner(hardwareMap, l, line, 0, LineRunner.HeadingType.ConstantHeadingVelo);
        waitForStart();
        tr.start();
        while (opModeIsActive() && !isStopRequested()) {
            if (tr.currentState != TrajectoryRunner.State.FINISHED) {
                tr.update();
            }
            l.update();
            telemetry.update();
        }
    }
}
