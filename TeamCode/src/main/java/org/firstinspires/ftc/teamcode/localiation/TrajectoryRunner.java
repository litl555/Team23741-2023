package org.firstinspires.ftc.teamcode.localiation;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.util.DashboardUtil;

import java.util.ArrayList;
import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.localiation.Constants.*;

@Config
/**
 * Handles the running of individual trajectories
 */
public class TrajectoryRunner {
    VoltageSensor battery;

    public enum State {
        RUNNING,
        CORRECTING,
        FINISHED
    }

    public enum HeadingType {
        ConstantHeadingVelo,
        TangentHeading
    }

    public int ind;
    public static double kv = .00055;
    public static double xyTolerance = 5;
    public static double aTolerance = Math.toRadians(2.5);
    public static double angleDes = 90;
    public State currentState = State.RUNNING;
    public static double ka = .0001;
    public static double kpxy = .005;
    public static double ks = .0001;
    public static double kp = .001;
    public static double kpa = -1;


    private double lastTime = 0;
    BasicFeedforward fx = new BasicFeedforward(new FeedforwardCoefficients(kv, ka, ks));
    BasicFeedforward fy = new BasicFeedforward(new FeedforwardCoefficients(kv, ka, ks));
    public Trajectory t;
    private double startTime;
    private CustomLocalization l;
    HeadingType headingType;
    LoggerTool loggerTool;

    public TrajectoryRunner(HardwareMap hardwareMap, CustomLocalization l, Trajectory trajectory, double angle, HeadingType headingType, LoggerTool telemetry) {
        battery = hardwareMap.voltageSensor.iterator().next();
        telemetry.setCurrentTrajectory(trajectory);
        loggerTool = telemetry;
        t = trajectory;
        this.headingType = headingType;
        this.l = l;
        this.angleDes = angle;

    }

    public void start() {
        startTime = toSec(getTime());
        lastTime = startTime;
    }

    public void update() {
        if (currentState == State.RUNNING) {
            runningMode();
        } else if (currentState == State.CORRECTING) {
            correctMode();
        }
    }

    private void runningMode() {
        ind = getIndex();
        double tv = t.velosSpaced.get(ind);
        Pose2d velocity = t.velocities(tv);

        Pose2d acceleration = t.accelerrations(tv);
        Pose2d velocityNormalized = t.normalize(velocity).times(t.mp.get(ind)).times(12.0 / battery.getVoltage());
        Pose2d accelerationNormalized = t.normalize(acceleration).times(t.amp.get(ind)).times(12.0 / battery.getVoltage());
        Pose2d positions = t.equation(tv);
        double loopTime = getLoopTime();
        double x = kpxy * (positions.getX() + Constants.robotPose.getY()) + (kp * (velocityNormalized.getX() + Constants.robotPose.minus(Constants.lastPose).div(loopTime).getY()) + (fx.calculate(0, velocityNormalized.getX(), accelerationNormalized.getX())));
        double y = -1.0 * (kpxy * (positions.getY() - Constants.robotPose.getX()) + kp * (velocityNormalized.getY() - Constants.robotPose.minus(Constants.lastPose).div(loopTime).getX()) + fy.calculate(0, velocityNormalized.getY(), accelerationNormalized.getY()));

        double angleVal = getAngleValue(velocityNormalized);

        l.setWeightedDrivePowers(new Pose2d(Math.cos(Constants.angle) * x - Math.sin(Constants.angle) * y, x * Math.sin(Constants.angle) + y * Math.cos(Constants.angle), angleVal));
        Constants.lastPose = Constants.robotPose;
        if (getElapsedTime() > t.totalTime) {
            currentState = State.CORRECTING;
        }
    }

    private void correctMode() {
        Pose2d positions = t.equation(1.0);
        double x = kpxy * (positions.getX() + robotPose.getY());
        double y = -1.0 * kpxy * (positions.getY() - robotPose.getX());
        l.setWeightedDrivePowers(new Pose2d(Math.cos(Constants.angle) * x - Math.sin(Constants.angle) * y, x * Math.sin(Constants.angle) + y * Math.cos(Constants.angle), kpa * (-Constants.angle - Math.toRadians(angleDes))));
        if (Math.sqrt(Math.pow(robotPose.getX() - positions.getY(), 2) + Math.pow(robotPose.getY() - positions.getX(), 2)) < xyTolerance && Math.abs(Constants.angle - Math.toRadians(angleDes)) < aTolerance) {
            currentState = State.FINISHED;
            l.setWeightedDrivePowers(new Pose2d(0, 0, 0));
            loggerTool.setCurrentTrajectoryNull();

        }
    }

    private double getAngleValue(Pose2d velocityNormalized) {
        double angleVal = 0;
        if (getElapsedTime() / t.totalTime < 1) {
            if (headingType == HeadingType.ConstantHeadingVelo) {
                angleVal = kpa * (-Constants.angle - getElapsedTime() / t.totalTime * Math.toRadians(angleDes));
            }
            if (headingType == HeadingType.TangentHeading) {
                if (velocityNormalized.getX() < 0 && velocityNormalized.getY() < 0) {
                    angleVal = kpa * (-Constants.angle - Math.atan2(velocityNormalized.getX(), velocityNormalized.getY()) + Math.PI);
                } else {
                    angleVal = kpa * (-Constants.angle - Math.atan2(velocityNormalized.getX(), velocityNormalized.getY()));
                }
            }

        } else {
            angleVal = kpa * (-Constants.angle - Math.toRadians(angleDes));
        }
        return angleVal;
    }

    private double getLoopTime() {
        double loop = toSec(getTime()) - lastTime;
        lastTime = toSec(getTime());
        return (loop);
    }

    private double getElapsedTime() {
        return toSec(Constants.getTime()) - startTime;
    }

    private int getIndex() {
        int masterIndex = 0;
        for (double time :
                t.timeValues) {
            if (getElapsedTime() < time) {
                masterIndex = t.timeValues.indexOf(time);
                break;
            }
        }
        if (masterIndex > 0) {
            masterIndex--;
        }
        return masterIndex;
    }
}
