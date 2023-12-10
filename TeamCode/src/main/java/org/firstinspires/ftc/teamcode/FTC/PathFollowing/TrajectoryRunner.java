package org.firstinspires.ftc.teamcode.FTC.PathFollowing;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

import static org.firstinspires.ftc.teamcode.FTC.Localization.Constants.angle;
import static org.firstinspires.ftc.teamcode.FTC.Localization.Constants.getTime;
import static org.firstinspires.ftc.teamcode.FTC.Localization.Constants.robotPose;
import static org.firstinspires.ftc.teamcode.FTC.Localization.Constants.toSec;
import static org.firstinspires.ftc.teamcode.FTC.PathFollowing.FollowerConstants.*;


@Config
/**
 * Handles the running of individual trajectories
 */
public class TrajectoryRunner {
    VoltageSensor battery;

    public enum State {
        PRESTART,
        RUNNING,
        CORRECTING,
        FINISHED
    }

    public enum HeadingType {
        ConstantHeadingVelo,
        TangentHeading
    }

    public static double speed = .25;
    public int ind;

    public static double angleDes = 90;
    public State currentState = State.PRESTART;
    int count = 0;

    private double lastTime = 0;

    BasicFeedforward fx = new BasicFeedforward(new FeedforwardCoefficients(kv, ka, ks));
    BasicFeedforward fy = new BasicFeedforward(new FeedforwardCoefficients(kv, ka, ks));
    public TrajectoryInterface t;
    private double startTime;
    private final CustomLocalization l;
    private double startAngle = 0.0;
    private double angleDes1 = 0.0;
    HeadingType headingType;
    double xerror;

    double yerror;
    int counter = 0;
    double xerrorLast = 0;
    double yerrorLast = 0;
    LoggerTool loggerTool;

    public TrajectoryRunner(HardwareMap hardwareMap, CustomLocalization l, TrajectoryInterface trajectory, double angle, HeadingType headingType, LoggerTool telemetry) {
        battery = hardwareMap.voltageSensor.iterator().next();

        loggerTool = telemetry;
        t = trajectory;
        this.headingType = headingType;
        this.l = l;
        angleDes = angle;

    }

    public void start() {
        loggerTool.setCurrentTrajectory(t);
        angleDes1 = -Constants.angle;
        startAngle = angleDes1;

        currentState = State.RUNNING;
        startTime = toSec(getTime());
        lastTime = startTime;
        loggerTool.add("HELLO", t.getTotalTime());
        loggerTool.add("start", startTime);

    }

    public void update() {
        if (Math.toRadians(angleDes) - startAngle > 0.0) {
            if (angleDes1 + angleSpeed > Math.toRadians(angleDes)) {
                angleDes1 = Math.toRadians(angleDes);
            } else {
                angleDes1 += angleSpeed;
            }
        }
        if (Math.toRadians(angleDes) - startAngle < 0.0) {
            if (angleDes1 - angleSpeed < Math.toRadians(angleDes)) {
                angleDes1 = Math.toRadians(angleDes);
            } else {
                angleDes1 -= angleSpeed;
            }
        }
        if (Math.toRadians(angleDes) == startAngle) {
            angleDes1 = Math.toRadians(angleDes);
        }
        loggerTool.add("current state", currentState);
        loggerTool.add("current state update", System.currentTimeMillis());
        if (currentState == State.RUNNING) {
            runningMode();
        } else if (currentState == State.CORRECTING) {
            correctMode();
        }
    }

    public int indexRunTime = 0;

    private void runningMode() {
        count++;
        if (count % 5 == 0) {
            double closestT = t.getClosestTValue(new Pose2d(-robotPose.getY(), robotPose.getX()));
            Robot.t = closestT;
            //loggerTool.add("t",closestT);
            if (closestT == 0.0) {
                closestT = .1;
            }
            Pose2d derivative = t.velocities(closestT);
            derivative = derivative.div(Math.sqrt(derivative.getX() * derivative.getX() + derivative.getY() * derivative.getY()));

            //loggerTool.drawPoint(t.equation(closestT));
            Robot.telemetry.add("amgle", Constants.angle);
            Robot.telemetry.add("angleDes", angleDes);
            //loggerTool.add("der",derivative);

            if (closestT == 1.0) {
                derivative = new Pose2d(0, 0, 0);
            }


            //loggerTool.add("radius",Math.sqrt(Math.abs(t.getCentripetalForceVector(closestT).dot(t.getCentripetalForceVector(closestT)))));
            Vector2d pathVector = new Vector2d(derivative.getX() * kvFollow, kvFollow * -1.0 * derivative.getY());
            //loggerTool.add("path",pathVector);
            Pose2d point = t.equation(closestT);
            //loggerTool.add("closestPoint",point);
            Vector2d centripetalForce = t.getCentripetalForceVector(closestT);
            centripetalForce = new Vector2d(centripetalForce.getX(), -centripetalForce.getY());
            Vector2d correctionalVector = new Vector2d(kvCorrect * (point.getX() + robotPose.getY()) + ka * (centripetalForce.getX()), ka * (centripetalForce.getY()) - kvCorrect * (point.getY() - robotPose.getX()));
            //loggerTool.add("xerror",point.getX() + robotPose.getY());
            //loggerTool.add("yerror",(point.getY() - robotPose.getX()));

            Vector2d sum = pathVector.plus(correctionalVector);
            if (Math.sqrt(sum.dot(sum)) > 1.0) {
                sum = sum.div(Math.sqrt(sum.dot(sum)));
            }
            sum = sum.times(speed);
            //l.setWeightedDrivePowers(new Pose2d(sum.getX(), sum.getY(), kpa * (angleDes - Constants.angle)));

            l.setWeightedDrivePowers(new Pose2d(Math.cos(Constants.angle) * sum.getX() - Math.sin(Constants.angle) * sum.getY(), sum.getX() * Math.sin(Constants.angle) + sum.getY() * Math.cos(Constants.angle), kpa * (-Constants.angle - angleDes1)));

            loggerTool.add("loop", getLoopTime());
            Vector2d robot = new Vector2d(-robotPose.getY(), robotPose.getX());
            if (robot.distTo(t.getEnd().vec()) < 200) {// && Math.abs(angleDes - Constants.angle) < Math.toRadians(10)
                if (!t.getEndStopped()) {
                    currentState = State.FINISHED;
                } else {
                    loggerTool.add("set to correcting", indexRunTime);
                    indexRunTime++;
                    currentState = State.CORRECTING;

                }
            }

            loggerTool.add("closetT from runningMode", t.equation(closestT));
//        ind = getIndex();
//        double tv = t.getVelosSpaced().get(ind);
//        Pose2d velocity = t.velocities(tv);
//        loggerTool.add("ind", ind);
//        Pose2d acceleration = t.accelerrations(tv);
//        Pose2d velocityNormalized = t.normalize(velocity).times(t.getMp().get(ind)).times(12.0 / battery.getVoltage());
//        Pose2d accelerationNormalized;
//        if (acceleration.getX() == 0 && acceleration.getY() == 0) {
//            accelerationNormalized = new Pose2d(0, 0);
//        } else {
//
//            accelerationNormalized = t.normalize(acceleration).times(t.getAmp().get(ind)).times(12.0 / battery.getVoltage());
//        }
//        Pose2d positions = t.equation(tv);
//        double loopTime = getLoopTime();
//        double x = kpxy * (positions.getX() + Constants.robotPose.getY()) + (kp * (velocityNormalized.getX() + Constants.robotPose.minus(Constants.lastPose).div(loopTime).getY()) + (fx.calculate(0, velocityNormalized.getX(), accelerationNormalized.getX())));
//        double y = -1.0 * (kpxy * (positions.getY() - Constants.robotPose.getX()) + kp * (velocityNormalized.getY() - Constants.robotPose.minus(Constants.lastPose).div(loopTime).getX()) + fy.calculate(0, velocityNormalized.getY(), accelerationNormalized.getY()));
//
//        double angleVal = getAngleValue(velocityNormalized);
//
//        l.setWeightedDrivePowers(new Pose2d(Math.cos(Constants.angle) * x - Math.sin(Constants.angle) * y, x * Math.sin(Constants.angle) + y * Math.cos(Constants.angle), angleVal));
//        loggerTool.add("totaltime", t.getTotalTime());
//        loggerTool.add("looptime", getLoopTime());
//
//        if (getElapsedTime() > t.getTotalTime()) {
//            if (t.getEndStopped()) {
//                currentState = State.CORRECTING;
//            } else {
//                currentState = State.FINISHED;
//                loggerTool.add("endElapsed", getElapsedTime());
//                loggerTool.add("end", Constants.toSec(Constants.getTime()));
//            }
//        }
        }
    }

    private void correctMode() {
        Pose2d positions = t.getEnd();
        double xerror = (positions.getX() + robotPose.getY());
        double yerror = positions.getY() - robotPose.getX();

        double x = speed * trajRunnerSpeedMult * xerror + dxy * (xerror - xerrorLast);
        double y = speed * -1.0 * trajRunnerSpeedMult * yerror + dxy * (yerror - yerrorLast);
        yerrorLast = yerror;
        xerrorLast = xerror;
        //l.setWeightedDrivePowers(new Pose2d(0,0,0));

        l.setWeightedDrivePowers(new Pose2d(Math.cos(Constants.angle) * x - Math.sin(Constants.angle) * y, x * Math.sin(Constants.angle) + y * Math.cos(Constants.angle), kpa * (-Constants.angle - angleDes1)));
        Constants.lastPose = Constants.robotPose;

        //loggerTool.add("error", Math.sqrt(Math.pow(robotPose.getX() - this.t.getEnd().getY(), 2) + Math.pow(robotPose.getY() - this.t.getEnd().getX(), 2)));
        //loggerTool.add("end", this.t.getEnd());
        //loggerTool.add("pos", robotPose);
        Robot.telemetry.add("error", Math.sqrt(Math.pow(-robotPose.getX() + t.getEnd().getY(), 2) + Math.pow(robotPose.getY() + t.getEnd().getX(), 2)));
        Robot.telemetry.add("end", t.getEnd());
        Robot.telemetry.add("pose", robotPose);
        Robot.telemetry.add("headingError", angle + angleDes);
        if (Math.sqrt(Math.pow(-robotPose.getX() + t.getEnd().getY(), 2) + Math.pow(robotPose.getY() + t.getEnd().getX(), 2)) < xyTolerance && Math.abs(Constants.angle + Math.toRadians(angleDes)) < aTolerance) {

            if (counter > 10) {
                currentState = State.FINISHED;
            }

            counter++;
            l.setWeightedDrivePowers(new Pose2d(0, 0, 0));
            loggerTool.setCurrentTrajectoryNull();

        } else {
            counter = 0;
        }

        loggerTool.add("last correct mode run", System.currentTimeMillis());
    }

    private double getAngleValue(Pose2d velocityNormalized) {
        double angleVal = 0;
        if (getElapsedTime() / t.getTotalTime() < 1) {
            if (headingType == HeadingType.ConstantHeadingVelo) {
                angleVal = kpa * (-Constants.angle - getElapsedTime() / t.getTotalTime() * Math.toRadians(angleDes));
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

    public double getLoopTime() {
        double loop = toSec(getTime()) - lastTime;
        lastTime = toSec(getTime());
        return (loop);
    }

    public double getElapsedTime() {
        return toSec(Constants.getTime()) - startTime;
    }

    private int getIndex() {
        int masterIndex = 0;
        for (double time :
                t.getTimeValuesVar()) {
            if (getElapsedTime() < time) {
                masterIndex = t.getTimeValuesVar().indexOf(time);
                break;
            }
        }
        if (masterIndex > 0) {
            masterIndex--;
        }
        return masterIndex;
    }
}
