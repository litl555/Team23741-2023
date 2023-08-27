package org.firstinspires.ftc.teamcode.localiation;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.DashboardUtil;

import java.util.ArrayList;
import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.localiation.Constants.*;

@Config
public class TrajectoryRunner {
    public static double kv = .00075;
    public static double angleDes = 90;
    public static double ka = .0001;
    public static double kpxy = .003;
    public static double ks = .0001;
    public static double kp = .001;
    public static double kpa = -1;

    private double lastTime = 0;
    BasicFeedforward fx = new BasicFeedforward(new FeedforwardCoefficients(kv, ka, ks));
    BasicFeedforward fy = new BasicFeedforward(new FeedforwardCoefficients(kv, ka, ks));
    Trajectory t;
    private double angle;
    private double startTime;
    private double[] xvals = new double[101];
    private double[] yvals = new double[101];

    private ArrayList<Double> xPosVals = new ArrayList<Double>();
    private ArrayList<Double> yPosVals = new ArrayList<Double>();
    private double[] xPos;
    private double[] yPos;

    private CustomLocalization l;
    private FtcDashboard dash = FtcDashboard.getInstance();

    public TrajectoryRunner(CustomLocalization l, Trajectory trajectory, double angle) {
        t = trajectory;
        this.l = l;
        this.angleDes = angle;
        for (int i = 0; i < 100; i++) {
            Pose2d pos = trajectory.equation((double) i / 100.0);
            yvals[i] = -(pos.getX() / 25.4);
            xvals[i] = (pos.getY() / 25.4);
        }
    }

    public void start() {
        generateTrajVals();
        startTime = toSec(getTime());
        lastTime = startTime;
    }

    private void generateTrajVals() {
        for (int i = 0; i < 100; i++) {
            Pose2d pos = t.equation((double) i / 100.0);
            yvals[i] = -(pos.getX() / 25.4);
            xvals[i] = (pos.getY() / 25.4);
        }
    }

    private void updatePosVals() {
        Pose2d rp = new Pose2d(Constants.robotPose.getX() * .0394, Constants.robotPose.getY() * .0394, (Constants.robotPose.getHeading()));
        xPosVals.add(0, (double) rp.getX());
        if (xPosVals.size() > 500) {
            xPosVals.remove(xPosVals.size() - 1);
        }
        yPosVals.add(0, (double) rp.getY());
        if (yPosVals.size() > 500) {
            yPosVals.remove(yPosVals.size() - 1);
        }
        double[] xArray = new double[xPosVals.size()];
        double[] yArray = new double[xPosVals.size()];
        for (int i = 0; i < xPosVals.size(); i++) {
            xArray[i] = xPosVals.get(i);

        }
        for (int i = 0; i < yPosVals.size(); i++) {
            yArray[i] = yPosVals.get(i);

        }
        xPos = xArray;
        yPos = yArray;
    }

    public void update() {
        updatePosVals();
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().strokePolyline(xvals, yvals);
        packet.fieldOverlay().strokePolyline(xPos, yPos);

        DashboardUtil.drawRobot(packet.fieldOverlay(), new Pose2d(Constants.robotPose.getX() * .0394, Constants.robotPose.getY() * .0394, (Constants.robotPose.getHeading())));
        dash.sendTelemetryPacket(packet);
        int ind = getIndex();
        double tv = t.velosSpaced.get(ind);
        Pose2d velocity = t.velocities(tv);

        Pose2d acceleration = t.accelerrations(tv);
        Pose2d velocityNormalized = t.normalize(velocity).times(t.mp.get(ind));
        Pose2d accelerationNormalized = t.normalize(acceleration).times(t.amp.get(ind));
        Pose2d positions = t.equation(tv);
        double loopTime = getLoopTime();
        double x = kpxy * (positions.getX() + Constants.robotPose.getY()) + (kp * (velocityNormalized.getX() + Constants.robotPose.minus(Constants.lastPose).div(loopTime).getY()) + (fx.calculate(0, velocityNormalized.getX(), accelerationNormalized.getX())));
        double y = -1.0 * (kpxy * (positions.getY() - Constants.robotPose.getX()) + kp * (velocityNormalized.getY() - Constants.robotPose.minus(Constants.lastPose).div(loopTime).getX()) + fy.calculate(0, velocityNormalized.getY(), accelerationNormalized.getY()));
        double angleVal = 0;
        if (getElapsedTime() / t.totalTime < 1) {
            angleVal = kpa * (-Constants.angle - getElapsedTime() / t.totalTime * Math.toRadians(angleDes));
        } else {
            angleVal = kpa * (-Constants.angle - Math.toRadians(angleDes));
        }
        l.setWeightedDrivePowers(new Pose2d(Math.cos(Constants.angle) * x - Math.sin(Constants.angle) * y, x * Math.sin(Constants.angle) + y * Math.cos(Constants.angle), angleVal));
        Constants.lastPose = Constants.robotPose;
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
