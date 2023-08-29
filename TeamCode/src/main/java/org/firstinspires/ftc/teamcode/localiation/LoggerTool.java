package org.firstinspires.ftc.teamcode.localiation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.DashboardUtil;

import java.util.ArrayList;

/**
 * Custom telemetry class to reduce clutter
 */
public class LoggerTool {
    private TelemetryPacket p = new TelemetryPacket();
    private final FtcDashboard dash = FtcDashboard.getInstance();
    private final ArrayList<Double> xPosVals = new ArrayList<>();
    private final ArrayList<Double> yPosVals = new ArrayList<>();
    private double[] xvals;
    private double[] yvals;

    public void add(String name, Object output) {
        p.put(name, output);
    }

    public void update() {
        drawPoseHistory();
        if (!getTrajectoryNull()) drawTrajectory();
        drawRobot();
        dash.sendTelemetryPacket(p);
        p = new TelemetryPacket();
    }

    public void setCurrentTrajectory(Trajectory trajectory) {
        xvals = new double[101];
        yvals = new double[101];
        for (int i = 0; i < 100; i++) {
            Pose2d pos = trajectory.equation((double) i / 100.0);
            yvals[i] = -(pos.getX() / 25.4);
            xvals[i] = (pos.getY() / 25.4);
        }
    }

    private void drawRobot() {
        p.fieldOverlay().setStroke("blue");
        DashboardUtil.drawRobot(p.fieldOverlay(), new Pose2d(Constants.robotPose.getX() * .0394, Constants.robotPose.getY() * .0394, (Constants.robotPose.getHeading())));
    }

    private void drawTrajectory() {
        p.fieldOverlay().setStroke("green");
        p.fieldOverlay().strokePolyline(xvals, yvals);
    }

    public void setCurrentTrajectoryNull() {
        xvals = null;
        yvals = null;
    }

    private boolean getTrajectoryNull() {
        return (xvals == null);
    }

    private void drawPoseHistory() {
        Canvas c = p.fieldOverlay();
        c.setStroke("red");
        updatePreviousPoseValsList();
        c.strokePolyline(convertListToArray(xPosVals), convertListToArray(yPosVals));
    }

    private void updatePreviousPoseValsList() {
        Pose2d rp = new Pose2d(Constants.robotPose.getX() * .0394, Constants.robotPose.getY() * .0394, (Constants.robotPose.getHeading()));
        xPosVals.add(0, (double) rp.getX());
        if (xPosVals.size() > 500) {
            xPosVals.remove(xPosVals.size() - 1);
        }
        yPosVals.add(0, (double) rp.getY());
        if (yPosVals.size() > 500) {
            yPosVals.remove(yPosVals.size() - 1);
        }
    }

    private double[] convertListToArray(ArrayList<Double> list) {
        double[] xArray = new double[list.size()];

        for (int i = 0; i < list.size(); i++) {
            xArray[i] = list.get(i);

        }

        return (xArray);
    }


}
