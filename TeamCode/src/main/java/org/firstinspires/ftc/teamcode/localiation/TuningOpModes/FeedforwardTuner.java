package org.firstinspires.ftc.teamcode.localiation.TuningOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.localiation.Constants;
import org.firstinspires.ftc.teamcode.localiation.CustomLocalization;
import org.firstinspires.ftc.teamcode.localiation.Line;
import org.firstinspires.ftc.teamcode.localiation.LineRunner;
import org.firstinspires.ftc.teamcode.localiation.LoggerTool;
import org.firstinspires.ftc.teamcode.localiation.Trajectory;
import org.firstinspires.ftc.teamcode.localiation.TrajectoryRunner;

import static org.firstinspires.ftc.teamcode.localiation.Constants.getTime;
import static org.firstinspires.ftc.teamcode.localiation.Constants.toSec;

@TeleOp
@Config
public class FeedforwardTuner extends LinearOpMode {
    enum Mode {
        AUTO,
        DRVER
    }

    public static double dist = 1000;

    @Override
    public void runOpMode() throws InterruptedException {
        Mode mode = Mode.AUTO;
        LoggerTool telemetry = new LoggerTool();
        CustomLocalization localization = new CustomLocalization(Constants.startPose, hardwareMap);
        Line trajectory = new Line(new Pose2d(0, 0), new Pose2d(1, dist));
        Line trajectory1 = new Line(new Pose2d(1, dist), new Pose2d(0, 0));
        LineRunner tr = new LineRunner(hardwareMap, localization, trajectory, 0, LineRunner.HeadingType.ConstantHeadingVelo);
        LineRunner tr1 = new LineRunner(hardwareMap, localization, trajectory1, 0, LineRunner.HeadingType.ConstantHeadingVelo);
        LineRunner trCurrent = tr;
        waitForStart();
        double lastTime = toSec(getTime());
        tr.start();

        while (opModeIsActive() && !isStopRequested()) {
            if (mode == Mode.AUTO) {
                telemetry.add("mp", tr.t.mp);
                telemetry.add("velocityx", trCurrent.t.normalize(trCurrent.t.velocity()).times(12.0 / trCurrent.battery.getVoltage()).times(trCurrent.t.mp.get(trCurrent.ind)).getX());
                telemetry.add("velocityxReal", Constants.robotPose.minus(Constants.lastPose).div(toSec(getTime()) - lastTime).getX());
                telemetry.add("velocityyReal", Constants.robotPose.minus(Constants.lastPose).div(toSec(getTime()) - lastTime).getY());
                telemetry.add("velocityy", trCurrent.t.normalize(trCurrent.t.velocity()).times(12.0 / trCurrent.battery.getVoltage()).times(trCurrent.t.mp.get(trCurrent.ind)).getY());
                telemetry.add("loopTime", toSec(getTime()) - lastTime);
                telemetry.add("equation", trCurrent.t.equation(tr.t.velosSpaced.get(tr.ind)));
                telemetry.add("totaltime", trCurrent.t.getTotalTime());
                telemetry.add("elapsedTime", tr.getElapsedTime());
                telemetry.add("ind", tr.t.velosSpaced.get(tr.ind));
                telemetry.add("spacedVelos", tr.t.velosSpaced);

                lastTime = toSec(getTime());
                if (tr.currentState != TrajectoryRunner.State.FINISHED && tr.currentState != TrajectoryRunner.State.PRESTART) {
                    tr.update();
                } else if (tr.currentState == TrajectoryRunner.State.FINISHED) {
                    tr1.start();
                    trCurrent = tr1;
                    tr.currentState = TrajectoryRunner.State.PRESTART;
                }
                if (tr1.currentState != TrajectoryRunner.State.FINISHED && tr1.currentState != TrajectoryRunner.State.PRESTART) {
                    tr1.update();
                } else if (tr1.currentState == TrajectoryRunner.State.FINISHED) {
                    tr.start();
                    trCurrent = tr;
                    tr1.currentState = TrajectoryRunner.State.PRESTART;
                }

                if (gamepad1.y) {
                    mode = Mode.DRVER;
                    localization.setWeightedDrivePowers(new Pose2d(0, 0, 0));
                }

            } else {
                double x = gamepad1.left_stick_x;
                double y = gamepad1.left_stick_y;
                localization.setWeightedDrivePowers(new Pose2d(Math.cos(Constants.angle) * x - Math.sin(Constants.angle) * y, x * Math.sin(Constants.angle) + y * Math.cos(Constants.angle), gamepad1.right_stick_x));
                if (gamepad1.a) {
                    trCurrent = tr;
                    tr1.currentState = TrajectoryRunner.State.PRESTART;
                    tr.start();
                    mode = Mode.AUTO;
                    localization.setWeightedDrivePowers(new Pose2d(0, 0, 0));
                }
            }


            telemetry.update();
            localization.update();
        }
    }
}
