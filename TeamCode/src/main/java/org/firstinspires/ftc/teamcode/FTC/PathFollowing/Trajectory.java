package org.firstinspires.ftc.teamcode.FTC.PathFollowing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;

import java.lang.reflect.Array;
import java.util.ArrayList;

/**
 * Defines a quintic spline trajectory and generates its motion profile
 */
public class Trajectory implements TrajectoryInterface {
    private final double deltaT = .1;
    private final double spaceRes = 60.0;
    public double length = 0;
    public ArrayList<Double> tValues;
    public ArrayList<Double> mp;
    public ArrayList<Double> velosSpaced, timeValues, amp, xAccels;
    Pose2d start, end, startVelo, endVelo, startAccel, endAccel, p0, p1, p2, p3, p4, p5;
    public double totalTime = 0;
    boolean startStopped, endStopped;


    public Trajectory(Pose2d start, Pose2d end, Pose2d startVelo, Pose2d endVelo, Pose2d startAccel, Pose2d endAccel, boolean startStopped, boolean endStopped) {
        this.start = start;
        this.end = end;
        this.startVelo = startVelo;
        this.endVelo = endVelo;
        this.startAccel = startAccel;
        this.endAccel = endAccel;
        this.startStopped = startStopped;
        this.endStopped = endStopped;
        p0 = start;
        p5 = end;
        p1 = new Pose2d(startVelo.getX() / 5.0 + p0.getX(), startVelo.getY() / 5.0 + p0.getY());
        p2 = new Pose2d(startAccel.getX() / 20.0 + 2.0 * p1.getX() - p0.getX(), startAccel.getY() / 20.0 + 2.0 * p1.getY() - p0.getY());
        p4 = new Pose2d(p5.getX() - endVelo.getX() / 5.0, p5.getY() - endVelo.getY() / 5.0);
        p3 = new Pose2d(endAccel.getX() / 20.0 + 2.0 * p4.getX() - p5.getX(), endAccel.getY() / 20.0 + 2.0 * p4.getY() - p5.getY());
        //length = calculateLength(0, 1);
        //generateTValues();
        //mp = generateMotionProfile();
        //getTimeValues();
        //getTotalTimeSum();


    }

    public boolean getEndStopped() {
        return (endStopped);
    }

    private void getTotalTimeSum() {
        totalTime = timeValues.get(timeValues.size() - 2);
    }

    /*
        //Old Version
        private void generateTValues() {
            ArrayList<Double> values = new ArrayList<>();
            double previous = 0;
            for (int i = 1; i < getTotalTime() / deltaT; i++) {
                values.add(getTValue(i * deltaT, previous));
                previous = values.get(values.size() - 1);

            }
            int counter = 0;
            for (double i : values) {
                values.set(counter, values.get(counter) / values.get(values.size() - 1));
                counter++;
            }
            tValues = values;
        }
    */
    private double calculateLength(double a, double b) {
        double integral = 0.0;
        for (int i = 0; i < 100; i++) {
            Pose2d velocity = velocities((double) i * (b - a) / 100.0 + a);
            integral += 1.0 * (b - a) / 100.0 * (Math.sqrt(Math.pow(velocity.getX(), 2) + Math.pow(velocity.getY(), 2)));
        }
        return (integral);
    }
/*
    private double getTValue(double time, double previousT) {
        double arcLengthVelo = 1.0 / 2.0 * ((getVelocityProfile(time).getX() - getVelocityProfile(time - deltaT).getX()) / (deltaT)) * Math.pow(deltaT, 2) + getVelocityProfile(time - deltaT).getX() * deltaT;
        Constants.yes += arcLengthVelo;
        double X = previousT + .01;
        for (int i = 0; i < 10; i++) {
            double len = calculateLength(previousT, X);
            double slope = getSlope(X);
            X = (-1.0 * (len - arcLengthVelo) + X * slope) / slope;
        }
        return X;
    }
*/
    /**
     * Generates equally spaced t values for spline
     */
    private void generateSpacedValues() {
        double previousT = 0.0;
        velosSpaced = new ArrayList<>();
        for (int i = 0; i < length / spaceRes; i++) {

            double X = previousT + .001;
            for (int a = 0; a < 100; a++) {
                double len = calculateLength(previousT, X);
                double slope = getSlope(X);
                X = (-1.0 * (len - spaceRes) + X * slope) / slope;

            }
            previousT = X;
            velosSpaced.add(X);

        }

    }

    /**
     * Generates motion profile for trajectory
     *
     * @return Motion profile
     */

    public ArrayList<Double> generateMotionProfile() {
        generateSpacedValues();
        int count = 1;
        ArrayList<Double> profile = new ArrayList<>();
        //FORWARD PASS
        for (double i : velosSpaced) {
            if (count == 1) {
                if (startStopped) {
                    profile.add(0.0);
                } else {
                    profile.add(Constants.maxVelocty);
                }
                double xoption = 1000000000;
                double yoption = 1000000000;
                double noOption = Constants.maxVelocty;
                if ((Math.pow(normalize(velocities(velosSpaced.get(count))).times(Constants.maxVelocty).getX(), 2) - Math.pow(normalize(velocities(i)).times(0.0).getX(), 2)) / (2.0 * spaceRes) > Constants.maxAcceleration) {
                    xoption = Math.sqrt(2.0 * Constants.maxAcceleration * spaceRes) / Math.abs(normalize(velocities(velosSpaced.get(count))).getX());

                }
                if ((Math.pow(normalize(velocities(velosSpaced.get(count))).times(Constants.maxVelocty).getY(), 2) - Math.pow(normalize(velocities(i)).times(0.0).getY(), 2)) / (2.0 * spaceRes) > Constants.maxAcceleration) {
                    yoption = Math.sqrt(2.0 * Constants.maxAcceleration * spaceRes) / Math.abs(normalize(velocities(velosSpaced.get(count))).getY());
                }
                profile.add(Math.min(Math.min(xoption, yoption), noOption));


            } else if (count < velosSpaced.size()) {
                double xoption = Constants.maxVelocty;
                double yoption = Constants.maxVelocty;
                double noOption = Constants.maxVelocty;
                double xCount = normalize(velocities(velosSpaced.get(count))).times(Constants.maxVelocty).getX();
                double xI = normalize(velocities(i)).times(profile.get(count - 1)).getX();
                double xAccel = (Math.signum(xCount) * Math.pow(xCount, 2) - Math.signum(xI) * Math.pow(xI, 2)) / (2.0 * spaceRes);
                double yCount = normalize(velocities(velosSpaced.get(count))).times(Constants.maxVelocty).getY();
                double yI = normalize(velocities(i)).times(profile.get(count - 1)).getY();
                double yAccel = (Math.signum(yCount) * Math.pow(yCount, 2) - Math.signum(yI) * Math.pow(yI, 2)) / (2.0 * spaceRes);

                if (xCount > xI) {
                    if (xAccel > Constants.maxAcceleration) {
                        if (xI < 0) {
                            if ((xI * xI - 2.0 * Constants.maxAcceleration * spaceRes) < 0) {
                                double X = Math.pow(xI, 2) / (2.0 * Constants.maxAcceleration);

                                xoption = Math.abs(Math.sqrt(2.0 * Constants.maxAcceleration * (spaceRes - X)) / normalize(velocities(velosSpaced.get(count))).getX());///(normalize(velocities(velosSpaced.get(count))).getX())
                            } else {
                                xoption = Math.abs(Math.sqrt(Math.pow(xI, 2) - 2.0 * Constants.maxAcceleration * spaceRes) / normalize(velocities(velosSpaced.get(count))).getX());///normalize(velocities(velosSpaced.get(count))).getX()
                            }
                        } else {
                            xoption = Math.abs(Math.sqrt(Math.pow(xI, 2) + 2.0 * Constants.maxAcceleration * spaceRes) / normalize(velocities(velosSpaced.get(count))).getX());
                        }
                    } else {
                        xoption = Constants.maxVelocty;
                    }
                } else {
                    xoption = Constants.maxVelocty;
                }
                if (yCount > yI) {
                    if (yAccel > Constants.maxAcceleration) {
                        if (yI < 0) {
                            if ((yI * yI - 2.0 * Constants.maxAcceleration * spaceRes) < 0) {
                                double X = Math.pow(yI, 2) / (2.0 * Constants.maxAcceleration);
                                yoption = Math.abs(Math.sqrt(2.0 * Constants.maxAcceleration * (spaceRes - X)) / normalize(velocities(velosSpaced.get(count))).getY());///(normalize(velocities(velosSpaced.get(count))).getY())
                            } else {
                                yoption = Math.abs(1.0 * Math.sqrt(Math.pow(yI, 2) - 2.0 * Constants.maxAcceleration * spaceRes) / normalize(velocities(velosSpaced.get(count))).getY());///normalize(velocities(velosSpaced.get(count))).getY()
                            }
                        } else {
                            yoption = Math.abs(Math.sqrt(Math.pow(yI, 2) + 2.0 * Constants.maxAcceleration * spaceRes) / normalize(velocities(velosSpaced.get(count))).getY());
                        }
                    } else {
                        yoption = Constants.maxVelocty;
                    }
                } else {
                    yoption = Constants.maxVelocty;
                }

                profile.add(Math.min(xoption, yoption));


                //profile.add(Math.min(Math.min(xoption, yoption), noOption));

            }
            count++;
        }
        count = profile.size() - 2;
        for (int i = profile.size() - 1; i > 0; i -= 1) {
            double xoption, yoption;

            if (i == profile.size() - 1) {
                if (endStopped) {
                    profile.set(i, 0.0);
                }
            }
            double xCount = normalize(velocities(velosSpaced.get(count))).times(profile.get(count)).getX();
            double xI = normalize(velocities(velosSpaced.get(i))).times(profile.get(i)).getX();
            double xAccel = (Math.signum(xCount) * Math.pow(xCount, 2) - Math.signum(xI) * Math.pow(xI, 2)) / (2.0 * spaceRes);

            double yCount = normalize(velocities(velosSpaced.get(count))).times(profile.get(count)).getY();
            double yI = normalize(velocities(velosSpaced.get(i))).times(profile.get(i)).getY();
            double yAccel = (Math.signum(yCount) * Math.pow(yCount, 2) - Math.signum(yI) * Math.pow(yI, 2)) / (2.0 * spaceRes);


            xoption = backPassCheck(xAccel, xCount, xI, count, normalize(velocities(velosSpaced.get(count))).getX());
            yoption = backPassCheck(yAccel, yCount, yI, count, normalize(velocities(velosSpaced.get(count))).getY());

            if (xoption < 1000000000 || yoption < 1000000000) {
                profile.set(count, Math.min(xoption, yoption));
            }
            count--;
        }
        //EDGE CASE PASS
        int counter1 = profile.size() - 2;
        for (int s = profile.size() - 1; s > 0; s--) {
            double yCounter = normalize(velocities(velosSpaced.get(counter1))).times(profile.get(counter1)).getY();
            double yS = normalize(velocities(velosSpaced.get(s))).times(profile.get(s)).getY();
            double yAcceleration = (Math.signum(yCounter) * Math.pow(yCounter, 2) - Math.signum(yS) * Math.pow(yS, 2)) / (2.0 * spaceRes);
            if (yAcceleration > Constants.maxAcceleration && yCounter > yS && yS < 0 && yCounter > 0) {
                profile.set(s, 0.0);
            }
            counter1--;
        }
        counter1 = 1;
        for (int s = 0; s < profile.size() - 2; s++) {
            double yCounter = normalize(velocities(velosSpaced.get(counter1))).times(profile.get(counter1)).getY();
            double yS = normalize(velocities(velosSpaced.get(s))).times(profile.get(s)).getY();
            double yAcceleration = (Math.signum(yCounter) * Math.pow(yCounter, 2) - Math.signum(yS) * Math.pow(yS, 2)) / (2.0 * spaceRes);
            if (Math.abs(yAcceleration) > Constants.maxAcceleration && yCounter <= 0.0 && yS <= 0.0 && yCounter < yS) {
                profile.set(counter1, Math.abs(Math.sqrt(Math.pow(yS, 2) + 2.0 * spaceRes * Constants.maxAcceleration) / normalize(velocities(velosSpaced.get(counter1))).getY()));
            }
            counter1++;
        }
        counter1 = profile.size() - 2;
        for (int s = profile.size() - 1; s > 0; s--) {
            double xCounter = normalize(velocities(velosSpaced.get(counter1))).times(profile.get(counter1)).getX();
            double xS = normalize(velocities(velosSpaced.get(s))).times(profile.get(s)).getX();
            double xAcceleration = (Math.signum(xCounter) * Math.pow(xCounter, 2) - Math.signum(xS) * Math.pow(xS, 2)) / (2.0 * spaceRes);
            if (xAcceleration > Constants.maxAcceleration && xCounter > xS && xS < 0 && xCounter > 0) {
                profile.set(counter1, 0.0);
            }
            counter1--;
        }
        counter1 = 1;
        for (int s = 0; s < profile.size() - 2; s++) {
            double xCounter = normalize(velocities(velosSpaced.get(counter1))).times(profile.get(counter1)).getX();
            double xS = normalize(velocities(velosSpaced.get(s))).times(profile.get(s)).getX();
            double xAcceleration = (Math.signum(xCounter) * Math.pow(xCounter, 2) - Math.signum(xS) * Math.pow(xS, 2)) / (2.0 * spaceRes);
            if (Math.abs(xAcceleration) > Constants.maxAcceleration && xCounter < xS && xCounter <= 0.0 && xS <= 0.0) {
                profile.set(counter1, Math.abs(Math.sqrt(Math.pow(xS, 2) + 2.0 * spaceRes * Constants.maxAcceleration) / normalize(velocities(velosSpaced.get(counter1))).getX()));
            }
            counter1++;
        }

        xAccels = new ArrayList<>();
        //BACKWARDS PASS

        int counter3 = profile.size() - 2;
        for (int i = profile.size() - 1; i > 0; i--) {
            double xCount = normalize(velocities(velosSpaced.get(counter3))).times(profile.get(counter3)).getX();
            double xI = normalize(velocities(velosSpaced.get(i))).times(profile.get(i)).getX();
            double xAccel = (Math.signum(xCount) * Math.pow(xCount, 2) - Math.signum(xI) * Math.pow(xI, 2)) / (2.0 * spaceRes);

            double yCount = normalize(velocities(velosSpaced.get(counter3))).times(profile.get(counter3)).getY();
            double yI = normalize(velocities(velosSpaced.get(i))).times(profile.get(i)).getY();
            double yAccel = (Math.signum(yCount) * Math.pow(yCount, 2) - Math.signum(yI) * Math.pow(yI, 2)) / (2.0 * spaceRes);
            double xoption = 1000000000;
            double yoption = 1000000000;
            if (Math.abs(xAccel) > Constants.maxAcceleration && xCount < xI && xI <= 0 && xCount <= 0) {
                xoption = Math.abs(Math.sqrt(Math.pow(xI, 2) + 2.0 * Constants.maxAcceleration * spaceRes) / normalize(velocities(velosSpaced.get(counter3))).getX());
            }
            if (Math.abs(yAccel) > Constants.maxAcceleration && yCount < yI && yI <= 0 && yCount <= 0) {
                yoption = Math.abs(Math.sqrt(Math.pow(yI, 2) + 2.0 * Constants.maxAcceleration * spaceRes) / normalize(velocities(velosSpaced.get(counter3))).getY());
            }
            if (xoption != 1000000000 || yoption != 1000000000) {
                profile.set(counter3, Math.min(xoption, yoption));
            }
            counter3--;
        }
        count = profile.size() - 2;
        for (int i = profile.size() - 1; i > 0; i -= 1) {
            double xoption, yoption;

            if (i == profile.size() - 1) {
                if (endStopped) {
                    profile.set(i, 0.0);
                }
            }
            double xCount = normalize(velocities(velosSpaced.get(count))).times(profile.get(count)).getX();
            double xI = normalize(velocities(velosSpaced.get(i))).times(profile.get(i)).getX();
            double xAccel = (Math.signum(xCount) * Math.pow(xCount, 2) - Math.signum(xI) * Math.pow(xI, 2)) / (2.0 * spaceRes);

            double yCount = normalize(velocities(velosSpaced.get(count))).times(profile.get(count)).getY();
            double yI = normalize(velocities(velosSpaced.get(i))).times(profile.get(i)).getY();
            double yAccel = (Math.signum(yCount) * Math.pow(yCount, 2) - Math.signum(yI) * Math.pow(yI, 2)) / (2.0 * spaceRes);


            xoption = backPassCheck(xAccel, xCount, xI, count, normalize(velocities(velosSpaced.get(count))).getX());
            yoption = backPassCheck(yAccel, yCount, yI, count, normalize(velocities(velosSpaced.get(count))).getY());

            if (xoption < 1000000000 || yoption < 1000000000) {
                profile.set(count, Math.min(xoption, yoption));
            }
            count--;
        }

        amp = new ArrayList<>();
        count = 1;
        for (double vel : profile) {
            if (count < profile.size()) {
                amp.add((Math.pow(profile.get(count), 2) - Math.pow(vel, 2)) / (2.0 * spaceRes));
            }
            count++;
        }
        return profile;
    }

    public Pose2d getEnd() {
        return (p5);
    }

    public ArrayList<Double> getMp() {
        return mp;
    }

    public ArrayList<Double> getAmp() {
        return amp;
    }

    public ArrayList<Double> getTimeValuesVar() {
        return (timeValues);
    }

    private double backPassCheck(double xAccel, double xCount, double xI, int count, double div) {
        double xoption;

        if (xCount > xI && xAccel > Constants.maxAcceleration) {
            if (xI < 0) {
                if ((Math.pow(xI, 2) - 2.0 * Constants.maxAcceleration * spaceRes) < 0) {
                    double X = Math.pow(xI, 2) / (2.0 * Constants.maxAcceleration);
                    xoption = Math.abs(Math.sqrt(2.0 * Constants.maxAcceleration * (spaceRes - X)) / div);///(normalize(velocities(velosSpaced.get(count))).getX())
                } else {
                    xoption = Math.abs(Math.sqrt(Math.pow(xI, 2) - 2.0 * Constants.maxAcceleration * spaceRes) / div);///normalize(velocities(velosSpaced.get(count))).getX()
                }
            } else {
                xoption = Math.abs(Math.sqrt(Math.pow(xI, 2) + 2.0 * Constants.maxAcceleration * spaceRes) / div);
            }
        } else {
            xoption = 1000000000;
        }
        return xoption;
    }

    private void getTimeValues() {
        double totalTime = 0;
        double previousI = 0;
        ArrayList<Double> timeValues = new ArrayList<>();
        int count = 0;
        for (double i : mp) {
            if (count > 0) {
                timeValues.add(totalTime + 2.0 * spaceRes / (previousI + i));
                totalTime += 2.0 * spaceRes / (previousI + i);
                previousI = i;
            } else {
                timeValues.add(0.0);
            }
            count++;

        }
        this.timeValues = timeValues;

    }

    //    public double getTotalTime() {
//        return ((length - 1.0 / 2.0 * Constants.maxAcceleration * Math.pow(Constants.maxVelocty / Constants.maxAcceleration, 2) - (1.0 / 2.0 * Constants.maxAcceleration * Math.pow(Constants.maxVelocty / Constants.maxAcceleration, 2))) / Constants.maxVelocty + 2.0 * (Constants.maxVelocty / Constants.maxAcceleration));
//    }
    public ArrayList<Double> getVelosSpaced() {
        return (velosSpaced);
    }


    public double getTotalTime() {
        return totalTime;
    }

    public double getRadiusOfCurvature(double t) {
        Pose2d velocity = velocities(t);

        Pose2d acc = accelerrations(t);
        double numerator = Math.pow((Math.pow(velocity.getX(), 2) + Math.pow(velocity.getY(), 2)), 3.0 / 2.0);
        double denominator = (velocity.getX() * acc.getY() - velocity.getY() * acc.getX());
        return (Math.abs(numerator / denominator));
    }

    public Vector2d getRadiusVectorNormalized(double t) {
        Vector2d point1 = new Vector2d(velocities(t).getY(), -velocities(t).getX());
        point1 = point1.div(Math.sqrt(point1.dot(point1)));
        Vector2d point2 = new Vector2d(-velocities(t).getY(), velocities(t).getX());
        point2 = point2.div(Math.sqrt(point2.dot(point2)));
        Vector2d point = new Vector2d(0, 0);
        if (equation(t + .01).vec().distTo(equation(t).vec().plus(point1.times(getRadiusOfCurvature(t)))) > equation(t + .01).vec().distTo(equation(t).vec().plus(point2.times(getRadiusOfCurvature(t))))) {
            point = point2;
        } else {
            point = point1;
        }
        return (point);
    }

    public Vector2d getCentripetalForceVector(double t) {
        double radius = getRadiusOfCurvature(t);
        Vector2d b = velocities(t).vec();
        Vector2d velocityReal = new Vector2d(Constants.velocity.getY() * (double) -1.0, Constants.velocity.getX());
        Vector2d velocityV = b.times(velocityReal.dot(b) / Math.abs(b.dot(b)));
        double velocity = Math.sqrt(velocityV.dot(velocityV));
        double centripetalAcceleration = velocity * velocity / radius;
        Vector2d radiusVec = getRadiusVectorNormalized(t);
        return (radiusVec.times(centripetalAcceleration));

    }

    public double getClosestTValue(Pose2d point) {
        double t = 0.013;
        boolean finished = false;
        int res = 30;
        for (int o = 0; o < res; o++) {
            if (!finished) {
                if (getDistanceSecondDerivative((double) o * 1.0 / (double) res, point) > 1 && getDistanceDerivative((double) o * 1.0 / (double) res, point) < 0.0) {

                    if (t == .01) {
                        t = (double) o * 1.0 / (double) res;
                    }


                }
                if (getDistanceDerivative((double) o * 1.0 / (double) res, point) > 0.0) {
                    t = (double) (o - 1.0) / (double) res;
                    finished = true;

                }
            }
        }
        if (t == 0.0) {
            return 0.01;
        }
        if (t == .013) {
            if (getDistanceDerivative(0, point) < getDistanceDerivative(1.0, point)) {
                return 0.1;

            } else {
                return 1.0;
            }
        }
        return (t);
/*
        for (int i = 0; i < 100; i++) {

            double slope = getDistanceSecondDerivative(t, point);
            if(slope==0.0){
                return t;
            }
            t = -(double)getDistanceDerivative(t, point) / slope + t;

        }
        if(t<0.0){
            return(0.0);
        }
        else if(t>1.0){
            return(1.0);
        }
        else {
            return (t);
        }

 */
    }

    public double getDistance(double t, Pose2d point) {
        return (Math.sqrt(Math.pow(equation(t).getX() - point.getX(), 2) + Math.pow(equation(t).getY() - point.getY(), 2)));
    }

    public double getDistanceDerivative(double t, Pose2d point) {

        Pose2d velocity = velocities(t);
        double numerator = 2.0 * (point.getX() - equation(t).getX()) * velocity.getX() + 2.0 * (point.getY() - equation(t).getY()) * velocity.getY();
        double denom = -(double) 2.0 * Math.sqrt(Math.pow(point.getX() - equation(t).getX(), 2) + Math.pow(point.getY() - equation(t).getY(), 2));

        return (numerator / denom);
//        return (2.0 * (equation(t).getX() - point.getX()) * velocity.getX() + 2.0 * (equation(t).getY() - point.getY()) * velocity.getY());
    }

    public double getDistanceSecondDerivative(double t, Pose2d point) {
        double topDer = (double) 2.0 * (accelerrations(t).getX() * (point.getX() - equation(t).getX()) + (-velocities(t).getX()) * (velocities(t).getX())) + (double) 2.0 * (accelerrations(t).getY() * (point.getY() - equation(t).getY()) + (-velocities(t).getY()) * (velocities(t).getY()));
        double bottom = 2.0 * Math.sqrt(Math.pow(point.getX() - equation(t).getX(), 2) + Math.pow(point.getY() - equation(t).getY(), 2));
        double bottomDer = (double) 2.0 * getDistanceDerivative(t, point);
        double top = (double) 2.0 * (point.getX() - equation(t).getX()) * velocities(t).getX() + (double) 2.0 * (point.getY() - equation(t).getY()) * velocities(t).getY();
        return ((topDer * bottom - bottomDer * top) / (Math.pow(bottom, 2)) * -(double) 1.0);
//        Pose2d velocity = velocities(t);
//        Pose2d acc = accelerrations(t);
//        return (2.0 * acc.getX() * (equation(t).getX() - point.getX()) + Math.pow(velocity.getX(), 2) + 2.0 * acc.getY() * (velocity.getY() - point.getY()) + velocity.getY() * velocity.getY());

    }

    public Pose2d normalize(Pose2d vec) {
        double len = Math.sqrt(Math.pow(vec.getX(), 2) + Math.pow(vec.getY(), 2));
        return (vec.div(len));
    }

    /*
        public Pose2d getVelocityProfile(double time) {
            double totalTime = getTotalTime();
            Constants.timed = totalTime;
            if (time < Constants.maxVelocty / Constants.maxAcceleration) {
                return (new Pose2d(time * Constants.maxAcceleration, Constants.maxAcceleration));
            } else if (time < totalTime - Constants.maxVelocty / Constants.maxAcceleration) {
                return (new Pose2d(Constants.maxVelocty, 0));
            } else if (time < totalTime) {
                return (new Pose2d(-1.0 * Constants.maxAcceleration * ((time - (totalTime - Constants.maxVelocty / Constants.maxAcceleration))) + Constants.maxVelocty, -Constants.maxAcceleration));
            } else {
                return new Pose2d(0, 0);

            }
        }
    */
    private double getSlope(double t) {
        Pose2d velocities = velocities(t);
        return (Math.sqrt(Math.pow(velocities.getX(), 2) + Math.pow(velocities.getY(), 2)));
    }

    public Pose2d equation(double t) {
        return (new Pose2d(Math.pow((1.0 - t), 5.0) * p0.getX() + Math.pow((1.0 - t), 4) * t * 5.0 * p1.getX() + Math.pow((1.0 - t), 3) * t * t * 10.0 * p2.getX() + Math.pow((1.0 - t), 2) * t * t * t * 10.0 * p3.getX() + Math.pow((1.0 - t), 1) * t * t * t * t * 5.0 * p4.getX() + Math.pow(t, 5) * p5.getX(), Math.pow((1.0 - t), 5.0) * p0.getY() + Math.pow((1.0 - t), 4) * t * 5.0 * p1.getY() + Math.pow((1.0 - t), 3) * t * t * 10.0 * p2.getY() + Math.pow((1.0 - t), 2) * t * t * t * 10.0 * p3.getY() + Math.pow((1.0 - t), 1) * t * t * t * t * 5.0 * p4.getY() + Math.pow(t, 5) * p5.getY()));
    }

    public Pose2d velocities(double t) {
        double x = 5.0 * ((p5.getX() - 5.0 * p4.getX() + 10.0 * p3.getX() - 10.0 * p2.getX() + 5.0 * p1.getX() - p0.getX()) * Math.pow(t, 4) + (4.0 * p4.getX() - 16.0 * p3.getX() + 24.0 * p2.getX() - 16.0 * p1.getX() + 4.0 * p0.getX()) * Math.pow(t, 3) + (6.0 * p3.getX() - 18.0 * p2.getX() + 18.0 * p1.getX() - 6.0 * p0.getX()) * t * t + (4.0 * p2.getX() - 8.0 * p1.getX() + 4.0 * p0.getX()) * t + p1.getX() - p0.getX());
        double y = 5.0 * ((p5.getY() - 5.0 * p4.getY() + 10.0 * p3.getY() - 10.0 * p2.getY() + 5.0 * p1.getY() - p0.getY()) * Math.pow(t, 4) + (4.0 * p4.getY() - 16.0 * p3.getY() + 24.0 * p2.getY() - 16.0 * p1.getY() + 4.0 * p0.getY()) * Math.pow(t, 3) + (6.0 * p3.getY() - 18.0 * p2.getY() + 18.0 * p1.getY() - 6.0 * p0.getY()) * t * t + (4.0 * p2.getY() - 8.0 * p1.getY() + 4.0 * p0.getY()) * t + p1.getY() - p0.getY());

        return (new Pose2d(x, y));
//        return (new Pose2d(5.0 * ((p5.getX() - 5.0 * p4.getX() + 10.0 * p3.getX() - 10.0 * p2.getX() + 5.0 * p1.getX() - p0.getX()) * Math.pow(t, 4) + 4.0 * (p4.getX() - 4.0 * (p3.getX() + p1.getX()) + 6.0 * p2.getX() + p0.getX()) * Math.pow(t, 3) + 6.0 * (p3.getX() - 3.0 * p2.getX() + 3.0 * p1.getX() - p0.getX()) * t * t + 4.0 * (p2.getX() - 2.0 * p1.getX() + p0.getX()) * t + p1.getX() - p0.getX()), 5.0 * ((p5.getY() - 5.0 * p4.getY() + 10.0 * p3.getY() - 10.0 * p2.getY() + 5.0 * p1.getY() - p0.getY()) * Math.pow(t, 4) + 4.0 * (p4.getY() - 4.0 * (p3.getY() + p1.getY()) + 6.0 * p2.getY() + p0.getY()) * Math.pow(t, 3) + 6.0 * (p3.getY() - 3.0 * p2.getY() + 3.0 * p1.getY() - p0.getY()) * t * t + 4.0 * (p2.getY() - 2.0 * p1.getY() + p0.getY()) * t + p1.getY() - p0.getY())));
    }

    public Pose2d accelerrations(double t) {
        return (new Pose2d(20.0 * (p5.getX() - 5.0 * p4.getX() + 10.0 * p3.getX() - 10.0 * p2.getX() + 5.0 * p1.getX() - p0.getX()) * t * t * t + 60.0 * (p4.getX() - 4.0 * (p3.getX() + p1.getX()) + 6.0 * p2.getX() + p0.getX()) * t * t + 60.0 * (p3.getX() - 3.0 * p2.getX() + 3.0 * p1.getX() - p0.getX()) * t + 20.0 * (p2.getX() - 2.0 * p1.getX() + p0.getX()), 20.0 * (p5.getY() - 5.0 * p4.getY() + 10.0 * p3.getY() - 10.0 * p2.getY() + 5.0 * p1.getY() - p0.getY()) * t * t * t + 60.0 * (p4.getY() - 4.0 * (p3.getY() + p1.getY()) + 6.0 * p2.getY() + p0.getY()) * t * t + 60.0 * (p3.getY() - 3.0 * p2.getY() + 3.0 * p1.getY() - p0.getY()) * t + 20.0 * (p2.getY() - 2.0 * p1.getY() + p0.getY())
//:))
        ));
    }

    @Override
    public int hashCode() {
        // based on unitys vec4 hash function -> this should be good enough
        // this is also assuming pose2D has an actual hash function and when i looked at the source code they didnt....
        return hashPose2D(start) ^ (hashPose2D(end) << 2) ^ (hashPose2D(startVelo) >> 2) ^ (hashPose2D(endVelo) >> 1);
    }

    private int hashPose2D(Pose2d p) {
        // again based on unity vec2 hash
        // idk if pose2d has an actual hash function and when i looked at the source code they didnt....
        // int cause double hashing is a nightmare (add java doesnt allow double << int?????)
        int x = (int) p.getX();
        int y = (int) p.getY();

        return x ^ (y << 2);
    }
}
