package org.firstinspires.ftc.teamcode.localiation.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.localiation.Constants;
import org.firstinspires.ftc.teamcode.localiation.CustomLocalization;
import org.firstinspires.ftc.teamcode.localiation.LoggerTool;
import org.firstinspires.ftc.teamcode.localiation.Trajectory;
import org.firstinspires.ftc.teamcode.localiation.TrajectoryRunner;

import static org.firstinspires.ftc.teamcode.localiation.Constants.getTime;
import static org.firstinspires.ftc.teamcode.localiation.Constants.toSec;

@Autonomous
public class TrajectoryTester extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        LoggerTool telemetry = new LoggerTool();
        CustomLocalization localization = new CustomLocalization(Constants.startPose, hardwareMap);
//        Trajectory trajectory = new Trajectory(new Pose2d(0, 0), new Pose2d(1, 1000), new Pose2d(-800, 4800), new Pose2d(-1600, 1800), new Pose2d(-1530, 4120), new Pose2d(0, -200));
//        Trajectory trajectory = new Trajectory(new Pose2d(-100, 0), new Pose2d(100, 000), new Pose2d(00, 3000), new Pose2d(0, -3000), new Pose2d(-500, 0), new Pose2d(500, 0));
        Trajectory trajectory = new Trajectory(new Pose2d(-250, 0), new Pose2d(0, 1000), new Pose2d(00, 5000), new Pose2d(0, 3726), new Pose2d(-2236, 2854), new Pose2d(135, 1440));

        Trajectory trajectory1 = new Trajectory(new Pose2d(100, 0), new Pose2d(00, -200), new Pose2d(763, -370), new Pose2d(0, 200), new Pose2d(-965, 670), new Pose2d(1947, -1680));
        Trajectory trajectory2 = new Trajectory(new Pose2d(00, -200), new Pose2d(-100, 00), new Pose2d(-630, -380), new Pose2d(0, 200), new Pose2d(-1560, 1560), new Pose2d(-940, -1280));
        TrajectoryRunner tr = new TrajectoryRunner(hardwareMap, localization, trajectory, 0, TrajectoryRunner.HeadingType.ConstantHeadingVelo, telemetry);
        TrajectoryRunner tr1 = new TrajectoryRunner(hardwareMap, localization, trajectory1, 0, TrajectoryRunner.HeadingType.ConstantHeadingVelo, telemetry);
        TrajectoryRunner tr2 = new TrajectoryRunner(hardwareMap, localization, trajectory2, 0, TrajectoryRunner.HeadingType.ConstantHeadingVelo, telemetry);
        TrajectoryRunner trCurrent = tr;
        waitForStart();
        double start = toSec(getTime());
        while (toSec(getTime()) - start < 5) {
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
            double velocityY = tr.t.normalize(tr.t.velocities(tr.t.velosSpaced.get(tr.ind))).times(tr.t.mp.get(tr.ind)).getY();
            double velocityX = tr.t.normalize(tr.t.velocities(tr.t.velosSpaced.get(tr.ind))).times(tr.t.mp.get(tr.ind)).getX();

            telemetry.add("velocityx", tr.t.normalize(tr.t.velocities(tr.t.velosSpaced.get(tr.ind))).times(tr.t.mp.get(tr.ind)).getX());
            telemetry.add("velocityy", tr.t.normalize(tr.t.velocities(tr.t.velosSpaced.get(tr.ind))).times(tr.t.mp.get(tr.ind)).getY());
            telemetry.add("velocity", Math.sqrt(Math.pow(velocityY, 2) + Math.pow(velocityX, 2)));

            telemetry.add("loopTime", toSec(getTime()) - lastTime);
            lastTime = toSec(getTime());
            telemetry.update();
            localization.update();
        }
    }
}
