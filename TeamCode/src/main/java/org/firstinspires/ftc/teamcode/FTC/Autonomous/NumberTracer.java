package org.firstinspires.ftc.teamcode.FTC.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.MultipleTrajectoryRunner;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.Trajectory;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.TrajectoryRunner;

import java.util.ArrayList;

@Autonomous
public class NumberTracer extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ArrayList<TrajectoryRunner> trajectoryRunners = new ArrayList<>();
        LoggerTool telemetry = new LoggerTool();
        CustomLocalization l = new CustomLocalization(new Pose2d(0, -300, 0), hardwareMap);
        Trajectory tr = new Trajectory(new Pose2d(300, 0), new Pose2d(900, 1750), new Pose2d(670, 1580), new Pose2d(160, 2900), new Pose2d(6330, -4220), new Pose2d(4600, -3400), true, false);


        trajectoryRunners.add(new TrajectoryRunner(hardwareMap, l, tr, 0, TrajectoryRunner.HeadingType.ConstantHeadingVelo, telemetry));
        tr = new Trajectory(new Pose2d(900, 1750), new Pose2d(2700, 2400), new Pose2d(1490, 950), new Pose2d(950, 1350), new Pose2d(-160, 270), new Pose2d(0, 0), false, true);

        trajectoryRunners.add(new TrajectoryRunner(hardwareMap, l, tr, 0, TrajectoryRunner.HeadingType.ConstantHeadingVelo, telemetry));
//        tr = new Trajectory(new Pose2d(243, 320), new Pose2d(243, 640), new Pose2d(940, 16), new Pose2d(-620, -20), new Pose2d(-200, 500), new Pose2d(400, 1000));
//        trajectoryRunners.add(new TrajectoryRunner(hardwareMap, l, tr, 0, TrajectoryRunner.HeadingType.ConstantHeadingVelo, telemetry));

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
