package org.firstinspires.ftc.teamcode.localiation.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.localiation.Constants;
import org.firstinspires.ftc.teamcode.localiation.CustomLocalization;
import org.firstinspires.ftc.teamcode.localiation.LoggerTool;
import org.firstinspires.ftc.teamcode.localiation.MultipleTrajectoryRunner;
import org.firstinspires.ftc.teamcode.localiation.Trajectory;
import org.firstinspires.ftc.teamcode.localiation.TrajectoryRunner;

import java.util.ArrayList;

@Autonomous
public class NumberTracer extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ArrayList<TrajectoryRunner> trajectoryRunners = new ArrayList<>();
        LoggerTool telemetry = new LoggerTool();
        CustomLocalization l = new CustomLocalization(new Pose2d(640, 0, 0), hardwareMap);
        Trajectory tr = new Trajectory(new Pose2d(0, 640), new Pose2d(243, 0), new Pose2d(813, 326), new Pose2d(3000, 0), new Pose2d(900, 700), new Pose2d(25000, 30));

        trajectoryRunners.add(new TrajectoryRunner(hardwareMap, l, tr, 0, TrajectoryRunner.HeadingType.ConstantHeadingVelo, telemetry));
        tr = new Trajectory(new Pose2d(243, 0), new Pose2d(243, 320), new Pose2d(940, 16), new Pose2d(-620, -20), new Pose2d(-200, 500), new Pose2d(400, 1000));
        trajectoryRunners.add(new TrajectoryRunner(hardwareMap, l, tr, 0, TrajectoryRunner.HeadingType.ConstantHeadingVelo, telemetry));
        tr = new Trajectory(new Pose2d(243, 320), new Pose2d(243, 640), new Pose2d(940, 16), new Pose2d(-620, -20), new Pose2d(-200, 500), new Pose2d(400, 1000));
        trajectoryRunners.add(new TrajectoryRunner(hardwareMap, l, tr, 0, TrajectoryRunner.HeadingType.ConstantHeadingVelo, telemetry));

        MultipleTrajectoryRunner mtr = new MultipleTrajectoryRunner(trajectoryRunners);
        waitForStart();
        mtr.start();
        while (opModeIsActive() && !isStopRequested()) {
            if (!mtr.finished) {
                mtr.update();
            }
            l.update();
            telemetry.update();

        }
    }
}
