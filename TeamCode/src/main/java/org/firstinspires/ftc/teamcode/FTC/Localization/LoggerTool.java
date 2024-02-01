package org.firstinspires.ftc.teamcode.FTC.Localization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.sun.source.tree.Tree;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.Trajectory;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.TrajectoryInterface;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.TreeMap;
import java.util.TreeSet;

import static org.firstinspires.ftc.teamcode.FTC.Localization.Constants.robotPose;

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
    private HashMap<String, Object> unsortedData = new HashMap<>();
    private TreeMap<String, HashSet<LoggerData>> sortedData = new TreeMap<>();
    TrajectoryInterface current = null;
    Telemetry telemetry;

    public LoggerTool(Telemetry telemetry) {
        dash.clearTelemetry();
        this.telemetry = telemetry;
        dash.setTelemetryTransmissionInterval(100);
    }

    public synchronized void add(String name, Object output) {
        if (!Robot.onlyLogImportant) {
            unsortedData.put(name, output);
            telemetry.addData(name, output);
        }
    }

    public synchronized void addImportant(String name, Object output) {
        unsortedData.put(name, output);
        telemetry.addData(name, output);
    }

    public synchronized void add(LoggerData data) {
        if (!Robot.onlyLogImportant) {
            if (!sortedData.containsKey(data.section)) sortedData.put(data.section, new HashSet<>());
            HashSet<LoggerData> s = sortedData.get(data.section);
            s.remove(data); s.add(data);
            if (data.shouldLogToDriver) telemetry.addData("(" + data.section + ") " + data.name, data.value);
        }
    }

    public synchronized void addImportant(LoggerData data) {
        if (!sortedData.containsKey(data.section)) sortedData.put(data.section, new HashSet<>());
        HashSet<LoggerData> s = sortedData.get(data.section);
        s.remove(data); s.add(data);
        if (data.shouldLogToDriver) telemetry.addData("(" + data.section + ") " + data.name, data.value);
    }

    public synchronized void update() {
        if (Robot.onlyLogImportant) addImportant("LOGGING INFO", "onlyLogImportant is active");
        p = new TelemetryPacket();

        //drawPoseHistory(); // TODO: fix this so it doesnt send 50k values per cycle
        if (!getTrajectoryNull()) drawTrajectory();
        /*
        if (current != null) {
            drawRobot(current.equation(Robot.t));
            Vector2d vec = current.getCentripetalForceVector(Robot.t);

            drawRobot(new Pose2d(-robotPose.getY() + vec.getX(), robotPose.getX() + vec.getY(), robotPose.getHeading()));
        }
        */

        drawRobot(new Pose2d(-robotPose.getY(), robotPose.getX(), robotPose.getHeading()));

        StringBuilder out = new StringBuilder("<br><br><tt>");

        int lineCount = 60;
        for (String section : sortedData.keySet()) {
            int marker = (lineCount - section.length()) / 2 - 1; // off by one error on odd strings but who cares
            out.append(repeat("=", marker)).append(" ").append(section).append(" ").append(repeat("=", marker)).append("<br>");

            HashSet<LoggerData> data = sortedData.get(section);
            for (LoggerData ld : data) out.append(ld.name).append(": ").append(ld.value).append("<br>");
        }

        out.append(repeat("+", lineCount)).append("<br>");
        for (String key : unsortedData.keySet()) out.append(key).append(": ").append(unsortedData.get(key)).append("<br>");

        out.append("</tt>");

        p.put("DATA", out.toString());

        FtcDashboard.getInstance().sendTelemetryPacket(p);

        p = new TelemetryPacket();

        telemetry.addData("Telemetry updated on", System.currentTimeMillis());
        telemetry.update();
    }

    public synchronized void addError(String name, Exception e) {
        StringBuilder s = new StringBuilder(e.toString());
        for (StackTraceElement st : e.getStackTrace()) s.append("<br>").append(st.toString());

        s.append("<br><br>");

        // unique id via currentTimeMillis
        addImportant(new LoggerData(name + " (" + System.currentTimeMillis() + ")", s.toString(), "ERROR"));
    }

    private String repeat(String s, int n) { return new String(new char[n]).replace("\0", s); }

    public synchronized void setCurrentTrajectory(TrajectoryInterface trajectory) {
        current = trajectory;
        xvals = new double[30];
        yvals = new double[30];
        for (int i = 0; i < 30; i++) {
            Pose2d pos = trajectory.equation((double) i / 30.0);
            yvals[i] = -(pos.getX() / 25.4);
            xvals[i] = (pos.getY() / 25.4);
        }

    }

    public void drawRobot(Pose2d pose) {
        pose = new Pose2d(pose.getY() / 25.4, -pose.getX() / 25.4, pose.getHeading());
        p.fieldOverlay().setStroke("red");
        DashboardUtil.drawRobot(p.fieldOverlay(), pose);
    }

    private void drawRobot() {
        p.fieldOverlay().setStroke("blue");
        DashboardUtil.drawRobot(p.fieldOverlay(), new Pose2d(Constants.robotPose.getX() * .0394, Constants.robotPose.getY() * .0394, (Constants.robotPose.getHeading())));
    }

    public void drawPoint(Pose2d pose) {
        p.fieldOverlay().fillCircle(pose.getY() / 25.4, -pose.getX() / 25.4, 1);
    }

    private synchronized void drawTrajectory() {
        p.fieldOverlay().setStroke("green");
        p.fieldOverlay().strokePolyline(xvals, yvals);
    }

    public synchronized void setCurrentTrajectoryNull() {
        xvals = null;
        yvals = null;
    }

    private synchronized boolean getTrajectoryNull() {
        return (xvals == null);
    }

    private synchronized void drawPoseHistory() {
        Canvas c = p.fieldOverlay();
        c.setStroke("red");
        updatePreviousPoseValsList();
        c.strokePolyline(convertListToArray(xPosVals), convertListToArray(yPosVals));
    }

    private synchronized void updatePreviousPoseValsList() {
        Pose2d rp = new Pose2d(Constants.robotPose.getX() * .0394, Constants.robotPose.getY() * .0394, (Constants.robotPose.getHeading()));
        xPosVals.add(0, (double) rp.getX());
        if (xPosVals.size() > 10_000) {
            xPosVals.remove(xPosVals.size() - 1);
        }
        yPosVals.add(0, (double) rp.getY());
        if (yPosVals.size() > 10_000) {
            yPosVals.remove(yPosVals.size() - 1);
        }
    }

    // lol .toArray()
    private double[] convertListToArray(ArrayList<Double> list) {
        double[] xArray = new double[list.size()];

        for (int i = 0; i < list.size(); i++) {
            xArray[i] = list.get(i);

        }

        return (xArray);
    }


}
