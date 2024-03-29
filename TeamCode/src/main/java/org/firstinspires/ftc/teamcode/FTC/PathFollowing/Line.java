package org.firstinspires.ftc.teamcode.FTC.PathFollowing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;

import java.util.ArrayList;

public class Line implements TrajectoryInterface {
    private double spaceRes = 10.0;
    ArrayList<Double> amp;
    private Pose2d start, end;
    public ArrayList<Double> velosSpaced;
    ArrayList<Double> timeValues = new ArrayList<>();
    public ArrayList<Double> mp = new ArrayList<>();
    boolean startStopped;
    boolean endStopped;

    public Line(Pose2d start, Pose2d end, boolean startStopped, boolean endStopped) {
        this.startStopped = startStopped;
        this.endStopped = endStopped;
        this.start = start;
        this.end = end;
        velosSpaced = generateVelosSpaced();
        mp = generateLineMP();

        timeValues = generateTimeValues();

    }

    public void forceSetP5(Pose2d p) {}

    private ArrayList<Double> generateTimeValues() {
        double totalTime = 0;
        ArrayList<Double> tv = new ArrayList<>();
        for (int i = 0; i < mp.size() - 1; i++) {
            tv.add(totalTime + 2.0 * spaceRes / (mp.get(i) + mp.get(i + 1)));
            totalTime += 2.0 * spaceRes / (mp.get(i) + mp.get(i + 1));
        }
        return tv;
    }

    public double getClosestTValue(Pose2d point) {
        double slope = getSlope();
        return ((-1.0 * point.getX() / slope - point.getY() - slope * start.getX() + start.getY()) / (-1 / slope - slope));

    }

    public Vector2d getCentripetalForceVector(double t) {
        return new Vector2d(0, 0);
    }

    private ArrayList<Double> generateLineMP() {
        ArrayList<Double> mp1 = new ArrayList<>();


        for (int i = 0; i < velosSpaced.size() - 1; i++) {
            if (i == 0) {
                if (startStopped) {
                    mp1.add(0.0);
                } else {
                    mp1.add(Constants.maxVelocty);
                }

            }

            if (Math.sqrt(Math.pow(mp1.get(i), 2) + 2.0 * spaceRes * Constants.maxAcceleration) < Constants.maxVelocty) {
                mp1.add(Math.sqrt(Math.pow(mp1.get(i), 2) + 2.0 * spaceRes * Constants.maxAcceleration));
            } else {
                mp1.add(Constants.maxVelocty);
            }


        }
        int counter1 = mp1.size() - 2;
        for (int i = mp1.size() - 1; i > 1; i--) {
            if (i == mp1.size() - 1) {
                if (endStopped) {
                    mp1.set(i, 0.0);
                } else {
                    mp1.set(i, Constants.maxVelocty);
                }
            }

            if ((Math.pow(mp1.get(i - 1), 2) - Math.pow(mp1.get(i), 2)) / (2.0 * spaceRes) > Constants.maxAcceleration) {
                mp1.set(counter1, Math.sqrt(Math.pow(mp1.get(i), 2) + 2.0 * spaceRes * Constants.maxAcceleration));
            }


            counter1--;
        }
        amp = new ArrayList<>();
        int count = 1;
        for (double vel : mp1) {
            if (count < mp1.size()) {
                amp.add((Math.pow(mp1.get(count), 2) - Math.pow(vel, 2)) / (2.0 * spaceRes));
            }
            count++;
        }

        return mp1;
    }

    public Pose2d equation(double x) {
        //the 0 is arbitrary to satisfy the interface requirements
        if (velocities(0).getX() < 0 && velocities(0).getY() < 0) {
            return (new Pose2d(start.getX() - x, start.getY() - getSlope() * x));
        } else {
            return (new Pose2d(x + start.getX(), getSlope() * x + start.getY()));
        }
    }


    public ArrayList<Double> getVelosSpaced() {
        return velosSpaced;
    }


    public ArrayList<Double> getMp() {
        return mp;
    }


    public ArrayList<Double> getAmp() {
        return amp;
    }


    public boolean getEndStopped() {
        return endStopped;
    }


    public ArrayList<Double> getTimeValuesVar() {
        return timeValues;
    }



    public Pose2d accelerrations(double t) {
        return new Pose2d(0, 0);
    }

    private double xEven() {
        return (Math.sqrt(Math.pow(spaceRes, 2) / (1.0 + Math.pow(getSlope(), 2))));
    }

    private double length(double x) {
        return (Math.sqrt(Math.pow(x, 2) + Math.pow(getSlope() * x, 2)));
    }

    private ArrayList<Double> generateVelosSpaced() {
        ArrayList<Double> vals = new ArrayList<>();
        double X = xEven();
        while (X < Math.abs(end.getX() - start.getX())) {


            vals.add(X + xEven());
            X += xEven();

        }
        vals.add(end.getY() - start.getY());
        return vals;
    }

    private double getLength() {
        return (Math.sqrt(Math.pow(end.getX() - start.getX(), 2) + Math.pow(end.getY() - start.getY(), 2)));
    }

    public Pose2d velocities(double t) {
        return (normalize(new Pose2d(end.getX() - start.getX(), end.getY() - start.getY())));
    }

    private double getSlope() {
        return (velocities(0).getY() / velocities(0).getX());
    }

    public Pose2d normalize(Pose2d vec) {
        double len = Math.sqrt(Math.pow(vec.getX(), 2) + Math.pow(vec.getY(), 2));
        return (vec.div(len));
    }

    public double getTotalTime() {
        return (timeValues.get(timeValues.size() - 1));
    }


    public Pose2d getEnd() {
        return end;
    }

}
