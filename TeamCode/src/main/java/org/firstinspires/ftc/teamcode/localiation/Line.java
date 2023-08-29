package org.firstinspires.ftc.teamcode.localiation;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.checkerframework.checker.units.qual.A;

import java.util.ArrayList;

public class Line {
    private double spaceRes = 10.0;
    private Pose2d start, end;
    public ArrayList<Double> velosSpaced;
    ArrayList<Double> timeValues = new ArrayList<>();
    public ArrayList<Double> mp = new ArrayList<>();

    public Line(Pose2d start, Pose2d end) {
        this.start = start;
        this.end = end;
        velosSpaced = generateVelosSpaced();
        mp = generateLineMP();

        timeValues = generateTimeValues();
    }

    private ArrayList<Double> generateTimeValues() {
        double totalTime = 0;
        ArrayList<Double> tv = new ArrayList<>();
        for (int i = 0; i < mp.size() - 1; i++) {
            tv.add(totalTime + 2.0 * spaceRes / (mp.get(i) + mp.get(i + 1)));
            totalTime += 2.0 * spaceRes / (mp.get(i) + mp.get(i + 1));
        }
        return tv;
    }

    private ArrayList<Double> generateLineMP() {
        ArrayList<Double> mp1 = new ArrayList<>();


        for (int i = 0; i < velosSpaced.size() - 1; i++) {
            if (i == 0) {
                mp1.add(0.0);

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
                mp1.set(i, 0.0);
            }

            if ((Math.pow(mp1.get(i - 1), 2) - Math.pow(mp1.get(i), 2)) / (2.0 * spaceRes) > Constants.maxAcceleration) {
                mp1.set(counter1, Math.sqrt(Math.pow(mp1.get(i), 2) + 2.0 * spaceRes * Constants.maxAcceleration));
            }


            counter1--;
        }
        return mp1;
    }

    public Pose2d equation(double x) {
        if (velocity().getX() < 0 && velocity().getY() < 0) {
            return (new Pose2d(start.getX() - x, start.getY() - getSlope() * x));
        } else {
            return (new Pose2d(x + start.getX(), getSlope() * x + start.getY()));
        }
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
        vals.add(end.getX());
        return vals;
    }

    private double getLength() {
        return (Math.sqrt(Math.pow(end.getX() - start.getX(), 2) + Math.pow(end.getY() - start.getY(), 2)));
    }

    public Pose2d velocity() {
        return (normalize(new Pose2d(end.getX() - start.getX(), end.getY() - start.getY())));
    }

    private double getSlope() {
        return (velocity().getY() / velocity().getX());
    }

    public Pose2d normalize(Pose2d vec) {
        double len = Math.sqrt(Math.pow(vec.getX(), 2) + Math.pow(vec.getY(), 2));
        return (vec.div(len));
    }

    public double getTotalTime() {
        return (timeValues.get(timeValues.size() - 1));
    }

}
