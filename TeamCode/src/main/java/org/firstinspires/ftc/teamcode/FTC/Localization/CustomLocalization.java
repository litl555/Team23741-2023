package org.firstinspires.ftc.teamcode.FTC.Localization;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

/**
 * Handles the calculations of odometry and drivetrain commands
 */

public class CustomLocalization {

    public Pose2d pose = new Pose2d(0, 0, 0);

    OdometryModule leftPod, rightPod, backPod;
    DcMotor leftFront, leftRear, rightFront, rightRear;
    double dT, dF, dS, dX, dY, fx, fy, r1, r0, rd, ld, bd;
    double rightTotal = 0;
    int counter = 0;
    double leftTotal = 0;
    double loopStart = 0.0;
    double lastTime = 0.0;
    double startTime;
    double backT = 0;
    double X_MULTIPLIER = 1.00551;
    double Y_MULTIPLER = (double) 1.00197;
    Pose2d start;

    public CustomLocalization(Pose2d startPose, HardwareMap hardwareMap) {
        this.start = startPose;

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
        leftPod = new OdometryModule(hardwareMap.dcMotor.get("intake"));
        rightPod = new OdometryModule(hardwareMap.dcMotor.get("drone"));
        rightPod.reverse();

        backPod = new OdometryModule(hardwareMap.dcMotor.get("rightRear"));

        backPod.reset();
        rightPod.reset();
        leftPod.reset();
        Constants.angle=start.getHeading();
    }

    public Pose2d getPoseEstimate() {
        return (Constants.robotPose);
    }

    public void update() {
        rd = rightPod.tickDeltaMm;
        ld = leftPod.tickDeltaMm;
        bd = backPod.tickDeltaMm;

        if (counter == 0) {
            startTime = Constants.toSec(Constants.getTime());
        }
        Robot.telemetry.add("running loop", Constants.toSec(Constants.getTime()) - loopStart);
        loopStart = Constants.toSec(Constants.getTime());
        counter++;
        Robot.telemetry.add("loopTime", (Constants.toSec(Constants.getTime()) - startTime) / (double) counter);

        //dr.updatePoseEstimate();
        //rightTotal += rightPod.getDelta() / 25.4;
        //rightPod.update();
        //leftTotal += leftPod.getDelta() / 25.4;
        //leftPod.update();
        //Robot.telemetry.add("rightTotal",rightTotal);
        //Robot.telemetry.add("leftTotal",leftTotal);

        rightTotal += rd;
        leftTotal += ld;
        backT += bd;
        //calculateDeltaPos(rd,ld,bd);

        pose = pose.plus(calculateDeltaPos(rd, ld, bd));
        //pose = new Pose2d(dr.getPoseEstimate().getX() * (double) 25.4 + (double) start.getX(), dr.getPoseEstimate().getY() * (double) 25.4 + (double) start.getY(), dr.getPoseEstimate().getHeading()+start.getHeading());

        Pose2d veloVec = (pose.minus(Constants.robotPose)).div(Constants.toSec(Constants.getTime()) - lastTime);
        lastTime = Constants.toSec(Constants.getTime());
        Constants.velocity = veloVec;

        Constants.robotPose = pose;
//        if (Math.abs(Math.toDegrees(Constants.lastPose.getHeading() - Constants.robotPose.getHeading())) < 100) {
//            Constants.angle += Math.toRadians(Math.toDegrees(Constants.robotPose.getHeading() - Constants.lastPose.getHeading()));
//        } else {
//            if (Math.toDegrees(Constants.lastPose.getHeading()) > 300) {
//                Constants.angle += Math.toRadians(Math.toDegrees(Constants.robotPose.getHeading() - Constants.lastPose.getHeading()) + 360);
//            } else {
//                Constants.angle += Math.toRadians(Math.toDegrees(Constants.robotPose.getHeading() - Constants.lastPose.getHeading()) - 360);
//
//            }
//        }
        Constants.lastPose = pose;
        Robot.telemetry.add("ld", leftTotal);
        Robot.telemetry.add("rd", rightTotal);
        Robot.telemetry.add("bd", backT);
    }

    private Pose2d calculateDeltaPos(double R, double L, double B) {
        R *= X_MULTIPLIER;
        L *= X_MULTIPLIER;
        B *= Y_MULTIPLER;

        dT = ((double) R - (double) L) / (Constants.LATERAL_DISTANCE);//(r-l)/(ly-ry)
        Constants.angle += dT;
        double T = Constants.robotPose.getHeading();
        dF = (R + L) / (double) 2.0;
        dS = (B - Constants.PERPENDICULAR_X * dT);
        if (dT == 0.0) {
            dX = dF;
            dY = dS;
        } else {
            r0 = dF / dT;
            r1 = dS / dT;
            dX = r0 * Math.sin(dT) - r1 * ((double) 1.0 - Math.cos(dT));
            dY = r1 * Math.sin(dT) + r0 * ((double) 1.0 - Math.cos(dT));
        }
        fx = (double) (dX * Math.cos(T) - (double) dY * Math.sin(T));
        fy = (double) (dY * Math.cos(T) + (double) dX * Math.sin(T));

        return (new Pose2d(fx, fy, dT));
    }

    public void setWeightedDrivePowers(Pose2d pose) {
        double y = -pose.getY();
        double x = pose.getX() * Constants.strafeMult;
        double rx = pose.getHeading();
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        setMotorPowers((y + x + rx) / denominator, (y - x - rx) / denominator, (y - x + rx) / denominator, (y + x - rx) / denominator);
    }

    public void setMotorPowers(double fl, double fr, double bl, double br) {
        Robot.hardware.setDrivetrain(fl, fr, bl, br);
    }
}
