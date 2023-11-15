package org.firstinspires.ftc.teamcode.localiation;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Handles the calculations of odometry and drivetrain commands
 */
public class CustomLocalization {
    Pose2d pose = new Pose2d(0, 0, 0);

    OdometryModule leftPod, rightPod, backPod;
    DcMotor leftFront, leftRear, rightFront, rightRear;
    double dT, dF, dS, dX, dY, fx, fy, r1, r0, rd, ld, bd;
    double X_MULTIPLIER = .994;
    double Y_MULTIPLER = (double) 1;

    public CustomLocalization(Pose2d startPose, HardwareMap hardwareMap) {
        Constants.angle = 0;

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pose = startPose;
        Constants.robotPose = startPose;
        leftPod = new OdometryModule(hardwareMap.dcMotor.get("leftRear"));
        rightPod = new OdometryModule(hardwareMap.dcMotor.get("leftFront"));
        rightPod.reverse();
        backPod = new OdometryModule(hardwareMap.dcMotor.get("rightFront"));
        backPod.reverse();
        backPod.reset();
        rightPod.reset();
        leftPod.reset();
    }

    public Pose2d getPoseEstimate() {
        return (Constants.robotPose);
    }

    public void update() {
        rd = rightPod.getDelta();
        ld = leftPod.getDelta();
        bd = backPod.getDelta();
        pose = pose.plus(calculateDeltaPos(rd, ld, bd));
        Constants.robotPose = pose;
    }

    private Pose2d calculateDeltaPos(double R, double L, double B) {
        R *= X_MULTIPLIER;
        L *= X_MULTIPLIER;
        B *= Y_MULTIPLER;
        rightPod.update();
        leftPod.update();
        backPod.update();
        dT = ((double) R - (double) L) / (Constants.LATERAL_DISTANCE);//(r-l)/(ly-ry)
        if (dT == 0) {
            dT = (double) .0000000000001;
        }
        Constants.angle += dT;
        double T = Constants.robotPose.getHeading();
        dF = (R + L) / (double) 2;
        dS = (B - Constants.PERPENDICULAR_X * dT);
        r0 = dF / dT;
        r1 = dS / dT;
        dX = r0 * Math.sin(dT) - r1 * ((double) 1 - Math.cos(dT));
        dY = r1 * Math.sin(dT) + r0 * ((double) 1 - Math.cos(dT));
        fx = (double) (dX * Math.cos(T) - dY * Math.sin(T));
        fy = (double) (dY * Math.cos(T) + dX * Math.sin(T));

        return (new Pose2d(fx, fy, dT));
    }

    public void setWeightedDrivePowers(Pose2d pose) {
        double y = -pose.getY();
        double x = pose.getX() * 1.1;
        double rx = pose.getHeading();
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        setMotorPowers((y + x + rx) / denominator, (y - x - rx) / denominator, (y - x + rx) / denominator, (y + x - rx) / denominator);
    }

    public void setMotorPowers(double fl, double fr, double bl, double br) {
        leftRear.setPower(bl);
        leftFront.setPower(fl);
        rightFront.setPower(fr);
        rightRear.setPower(br);
    }
}
