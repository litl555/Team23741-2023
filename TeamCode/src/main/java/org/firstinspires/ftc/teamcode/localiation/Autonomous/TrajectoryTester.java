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
        Trajectory trajectory = new Trajectory(new Pose2d(0, 0), new Pose2d(0, 1000), new Pose2d(-800, 4800), new Pose2d(-1600, 1800), new Pose2d(-1530, 4120), new Pose2d(0, -200));
        TrajectoryRunner tr = new TrajectoryRunner(hardwareMap, localization, trajectory, 0, TrajectoryRunner.HeadingType.ConstantHeadingVelo, telemetry);

        waitForStart();
        double lastTime = toSec(getTime());
        tr.start();

        while (opModeIsActive() && !isStopRequested()) {
            if (tr.currentState != TrajectoryRunner.State.FINISHED) {
                tr.update();
            }
            telemetry.add("velocityx", tr.t.normalize(tr.t.velocities(tr.t.velosSpaced.get(tr.ind))).times(tr.t.mp.get(tr.ind)).getX());
            telemetry.add("velocityy", tr.t.normalize(tr.t.velocities(tr.t.velosSpaced.get(tr.ind))).times(tr.t.mp.get(tr.ind)).getY());
            telemetry.add("loopTime", toSec(getTime()) - lastTime);
            lastTime = toSec(getTime());
            telemetry.update();
            localization.updateMethod();
        }
    }
}
