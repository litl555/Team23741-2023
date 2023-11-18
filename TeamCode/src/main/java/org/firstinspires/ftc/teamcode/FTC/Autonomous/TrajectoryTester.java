package org.firstinspires.ftc.teamcode.FTC.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.Trajectory;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.TrajectoryRunner;

import static org.firstinspires.ftc.teamcode.FTC.Localization.Constants.getTime;
import static org.firstinspires.ftc.teamcode.FTC.Localization.Constants.toSec;

@Autonomous
public class TrajectoryTester extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        LoggerTool telemetry = new LoggerTool();
        CustomLocalization localization = new CustomLocalization(Constants.startPose, hardwareMap);
//        Trajectory trajectory = new Trajectory(new Pose2d(0, 0), new Pose2d(1, 1000), new Pose2d(-800, 4800), new Pose2d(-1600, 1800), new Pose2d(-1530, 4120), new Pose2d(0, -200));
//        Trajectory trajectory = new Trajectory(new Pose2d(-100, 0), new Pose2d(100, 000), new Pose2d(00, 3000), new Pose2d(0, -3000), new Pose2d(-500, 0), new Pose2d(500, 0));
        //Trajectory trajectory = new Trajectory(new Pose2d(-250, 0), new Pose2d(0, 1000), new Pose2d(00, 5000), new Pose2d(0, 3726), new Pose2d(-2236, 2854), new Pose2d(135, 1440));
        Trajectory trajectory = new Trajectory(new Pose2d(-250, 0), new Pose2d(290, 1760), new Pose2d(0, 5000), new Pose2d(1810, -3090), new Pose2d(-2236, 2854), new Pose2d(135, 1440), true, false);

        Trajectory trajectory1 = new Trajectory(new Pose2d(300, 0), new Pose2d(900, 1750), new Pose2d(670, 1580), new Pose2d(160, 2900), new Pose2d(6330, -4220), new Pose2d(4600, -3400), false, false);
        Trajectory trajectory2 = new Trajectory(new Pose2d(00, -200), new Pose2d(-100, 00), new Pose2d(-630, -380), new Pose2d(0, 200), new Pose2d(-1560, 1560), new Pose2d(-940, -1280), false, true);
        TrajectoryRunner tr = new TrajectoryRunner(hardwareMap, localization, trajectory, 0, TrajectoryRunner.HeadingType.ConstantHeadingVelo, telemetry);

        TrajectoryRunner tr1 = new TrajectoryRunner(hardwareMap, localization, trajectory1, 0, TrajectoryRunner.HeadingType.ConstantHeadingVelo, telemetry);
        TrajectoryRunner tr2 = new TrajectoryRunner(hardwareMap, localization, trajectory2, 0, TrajectoryRunner.HeadingType.ConstantHeadingVelo, telemetry);
        TrajectoryRunner trCurrent = tr;
        waitForStart();
        double start = toSec(getTime());
        while (toSec(getTime()) - start < 1) {
            telemetry.add("velocityx", 0.0);
            telemetry.add("velocityy", 0.0);
            telemetry.update();
            idle();
        }
        double lastTime = toSec(getTime());
        tr.start();

        while (opModeIsActive() && !isStopRequested()) {
//            if (tr.currentState != TrajectoryRunner.State.FINISHED && tr.currentState != TrajectoryRunner.State.PRESTART) {
//                tr.update();
//            } else if (tr.currentState == TrajectoryRunner.State.FINISHED) {
//                tr1.start();
//                trCurrent = tr1;
//                tr.currentState = TrajectoryRunner.State.PRESTART;
//            }
//            if (tr1.currentState != TrajectoryRunner.State.FINISHED && tr1.currentState != TrajectoryRunner.State.PRESTART) {
//                tr1.update();
//            } else if (tr1.currentState == TrajectoryRunner.State.FINISHED) {
//                tr2.start();
//                trCurrent = tr2;
//                tr1.currentState = TrajectoryRunner.State.PRESTART;
//            }
//            if (tr2.currentState != TrajectoryRunner.State.FINISHED && tr2.currentState != TrajectoryRunner.State.PRESTART) {
//                tr2.update();
//            } else if (tr1.currentState == TrajectoryRunner.State.FINISHED) {
//
//            }
            if (tr.currentState != TrajectoryRunner.State.FINISHED) {
                tr.update();
            }
            double velocityY = tr.t.normalize(tr.t.velocities(tr.t.getVelosSpaced().get(tr.ind))).times(tr.t.getMp().get(tr.ind)).getY();
            double velocityX = tr.t.normalize(tr.t.velocities(tr.t.getVelosSpaced().get(tr.ind))).times(tr.t.getMp().get(tr.ind)).getX();

            telemetry.add("velocityx", tr.t.normalize(tr.t.velocities(tr.t.getVelosSpaced().get(tr.ind))).times(tr.t.getMp().get(tr.ind)).getX());
            telemetry.add("velocityy", tr.t.normalize(tr.t.velocities(tr.t.getVelosSpaced().get(tr.ind))).times(tr.t.getMp().get(tr.ind)).getY());
            telemetry.add("velocity", Math.sqrt(Math.pow(velocityY, 2) + Math.pow(velocityX, 2)));

            telemetry.add("loopTime", toSec(getTime()) - lastTime);
            lastTime = toSec(getTime());
            telemetry.update();
            localization.update();
        }
    }
}
