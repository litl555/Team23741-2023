package org.firstinspires.ftc.teamcode.localiation;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.FeedforwardController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.FeedforwardEx;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Const;
@Config
@TeleOp
public class Test extends LinearOpMode {
    public static double kv=.00075;
    public static double ka=.0001;
    public static double ks=.0001;
    public static double dist=700;
    @Override
    public void runOpMode() throws InterruptedException {
        Trajectory trajectory=new Trajectory(new Pose2d(0,0),new Pose2d(0,800),new Pose2d(1010,330),new Pose2d(1000,700),new Pose2d(-390,264),new Pose2d(300,0));
        //Trajectory trajectory=new Trajectory(new Pose2d(0,0),new Pose2d(0,dist),new Pose2d(0,0),new Pose2d(0,dist),new Pose2d(0,0),new Pose2d(0,dist));
        FtcDashboard ftcDashboard=FtcDashboard.getInstance();
        CustomLocalization l=new CustomLocalization(new Pose2d(0,0,0),hardwareMap);
        BasicFeedforward fx=new BasicFeedforward(new FeedforwardCoefficients(kv,ka,ks));
        BasicFeedforward fy=new BasicFeedforward(new FeedforwardCoefficients(kv,ka,ks));
        double lastTime=0;
        Constants.lastPose=new Pose2d(0,0,0);
        waitForStart();
        double startTime=Constants.getTime()/100000000.0;

        while(Constants.getTime()/100000000.0-50.0<startTime){
            TelemetryPacket packet=new TelemetryPacket();
            packet.put("velocityNormalized",0.0);
            packet.put("velocityReal",0.0);
            ftcDashboard.sendTelemetryPacket(packet);

        }
        startTime=Constants.getTime()/100000000.0;

        while(opModeIsActive()&&!isStopRequested()){


            TelemetryPacket packet=new TelemetryPacket();
            double loopTime=Constants.getTime()/1000000.0-lastTime;
            packet.put("loop Time",loopTime);
            lastTime=Constants.getTime()/1000000.0;



            Pose2d velocity=trajectory.velocities(trajectory.tValues.get((int)Math.round(Constants.getTime()/100000000.0-startTime)));
            Pose2d acceleration=trajectory.accelerrations(trajectory.tValues.get((int)Math.round(Constants.getTime()/100000000.0-startTime)));
            Pose2d velocityNormalized=trajectory.normalize(velocity).times(trajectory.getVelocityProfile(Constants.getTime()/1000000000.0-startTime/10.0).getX());
            Pose2d accelerationNormalized=trajectory.normalize(acceleration).times(trajectory.getVelocityProfile(Constants.getTime()/1000000000.0-startTime/10.0).getY());

            l.setWeightedDrivePowers(new Pose2d(fx.calculate(0, velocityNormalized.getX(),accelerationNormalized.getX()),-1.0*fy.calculate(0,velocityNormalized.getY(),accelerationNormalized.getY()),0));
            if(Constants.robotPose.minus(Constants.lastPose).div(loopTime/1000.0).getX()>-10) {
                packet.put("velocityReal", Constants.robotPose.minus(Constants.lastPose).div(loopTime / 1000.0).getX());
            }
            Constants.lastPose=Constants.robotPose;
            l.updateMethod();
            packet.put("time",(int)Math.round(Constants.getTime()/100000000.0-startTime));
            packet.put("length",trajectory.length);
            packet.put("tvalue",trajectory.tValues.get((int)Math.round(Constants.getTime()/100000000.0-startTime)));
            packet.put("position",trajectory.equation(trajectory.tValues.get((int)Math.round(Constants.getTime()/100000000.0-startTime))));
            packet.put("velocity",trajectory.velocities(trajectory.tValues.get((int)Math.round(Constants.getTime()/100000000.0-startTime))));

            packet.put("velocityNormalized",velocityNormalized.getY());
            ftcDashboard.sendTelemetryPacket(packet);


        }
    }
}
