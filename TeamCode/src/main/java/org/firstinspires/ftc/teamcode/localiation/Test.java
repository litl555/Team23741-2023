package org.firstinspires.ftc.teamcode.localiation;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.DashboardUtil;

@Config
@TeleOp
public class Test extends LinearOpMode {
    public static double kv = .00075;
    public static double angleDes = 90;
    public static double ka = .0001;
    public static double kpxy = .003;
    public static double ks = .0001;
    public static double dist = 700;
    public static double kp = .001;
    public static double kpa = -1;
    @Override
    public void runOpMode() throws InterruptedException {
        //Trajectory trajectory = new Trajectory(new Pose2d(0, 0), new Pose2d(395, -560), new Pose2d(330, -1170), new Pose2d(1480, 1800), new Pose2d(3110, 1950), new Pose2d(-1280, -1250));
        //Trajectory trajectory = new Trajectory(new Pose2d(0, 0), new Pose2d(0, 1000), new Pose2d(740, 2560), new Pose2d(-1000, 3940), new Pose2d(1530, 780), new Pose2d(-750, -1305));

        Trajectory trajectory = new Trajectory(new Pose2d(0, 0), new Pose2d(0, 1000), new Pose2d(1560, 2610), new Pose2d(580, 2885), new Pose2d(3110, 1950), new Pose2d(-1280, -1250));
        //Trajectory trajectory=new Trajectory(new Pose2d(0,0),new Pose2d(0,dist),new Pose2d(0,0),new Pose2d(0,dist),new Pose2d(0,0),new Pose2d(0,dist));
        FtcDashboard ftcDashboard = FtcDashboard.getInstance();
        CustomLocalization l = new CustomLocalization(new Pose2d(0, 0, 0), hardwareMap);
        BasicFeedforward fx = new BasicFeedforward(new FeedforwardCoefficients(kv, ka, ks));
        BasicFeedforward fy = new BasicFeedforward(new FeedforwardCoefficients(kv, ka, ks));
        double lastTime = 0;
        Constants.lastPose = new Pose2d(0, 0, 0);
        double[] xvals = new double[100];
        double[] yvals = new double[100];

        for (int i = 0; i < 100; i++) {
            Pose2d pos = trajectory.equation((double) i / 100.0);
            yvals[i] = -(pos.getX() / 25.4);
            xvals[i] = (pos.getY() / 25.4);
        }
        waitForStart();
        double startTime = Constants.getTime() / 100000000.0;
        Constants.angle = 0;

        while (Constants.getTime() / 100000000.0 - 100.0 < startTime) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("velocityNormalized", 0.0);
            packet.put("velocityReal", 0.0);
            packet.put("velocityX", 0.0);
            packet.put("velocityY", 0.0);
            packet.put("positionX", 0.0);
            packet.put("postionY", 0.0);
            packet.put("accelX", 0.0);
            packet.put("accelY", 0.0);
            ftcDashboard.sendTelemetryPacket(packet);

        }
        startTime=Constants.getTime()/100000000.0;

        while(opModeIsActive()&&!isStopRequested()) {


            TelemetryPacket packet = new TelemetryPacket();
            double loopTime = Constants.getTime() / 1000000.0 - lastTime;
            //packet.put("loop Time", loopTime);
            lastTime = Constants.getTime() / 1000000.0;
            double elapsedTime = Constants.getTime() / 1000000000.0 - startTime / 10.0;
            int masterIndex = 0;
            //packet.put("times", trajectory.timeValues);
            for (double time :
                    trajectory.timeValues) {
                if (Constants.getTime() / 1000000000.0 - startTime / 10.0 < time) {
                    masterIndex = trajectory.timeValues.indexOf(time);
                    break;
                }


            }
            if (masterIndex > 0) {
                masterIndex--;
            }
            //packet.put("time", masterIndex);
            Pose2d velocity = trajectory.velocities(trajectory.velosSpaced.get(masterIndex));

            Pose2d acceleration = trajectory.accelerrations(trajectory.velosSpaced.get(masterIndex));
            Pose2d velocityNormalized = trajectory.normalize(velocity).times(trajectory.mp.get(masterIndex));
            Pose2d accelerationNormalized = trajectory.normalize(acceleration).times(trajectory.amp.get(masterIndex));
            Pose2d positions = trajectory.equation(trajectory.velosSpaced.get(masterIndex));
            double x = kpxy * (positions.getX() + Constants.robotPose.getY()) + (kp * (velocityNormalized.getX() + Constants.robotPose.minus(Constants.lastPose).div(loopTime / 1000.0).getY()) + (fx.calculate(0, velocityNormalized.getX(), accelerationNormalized.getX())));
            double y = -1.0 * (kpxy * (positions.getY() - Constants.robotPose.getX()) + kp * (velocityNormalized.getY() - Constants.robotPose.minus(Constants.lastPose).div(loopTime / 1000.0).getX()) + fy.calculate(0, velocityNormalized.getY(), accelerationNormalized.getY()));
            double angleVal = 0;
            if (elapsedTime / trajectory.totalTime < 1) {
                angleVal = kpa * (-Constants.angle - elapsedTime / trajectory.totalTime * Math.toRadians(angleDes));
            } else {
                angleVal = kpa * (-Constants.angle - Math.toRadians(angleDes));
            }
            l.setWeightedDrivePowers(new Pose2d(Math.cos(Constants.angle) * x - Math.sin(Constants.angle) * y, x * Math.sin(Constants.angle) + y * Math.cos(Constants.angle), angleVal));
            if (Constants.robotPose.minus(Constants.lastPose).div(loopTime / 1000.0).getX() > -10) {
                //packet.put("velocityReal", Constants.robotPose.minus(Constants.lastPose).div(loopTime / 1000.0).getX());
                //packet.put("velocityXReal", Constants.robotPose.minus(Constants.lastPose).div(loopTime / 1000.0).getY());

            }
            packet.put("velocityY", velocityNormalized.getY());
            packet.put("velocityX", velocityNormalized.getX());
//            packet.put("positionX",positions.getX());
//            packet.put("postionY",positions.getY());
//            packet.put("accelX",accelerationNormalized.getX());
//            packet.put("accelY",accelerationNormalized.getY());
            Constants.lastPose = Constants.robotPose;
            l.update();
//            packet.put("time",(int)Math.round(Constants.getTime()/100000000.0-startTime));
//            packet.put("length",trajectory.length);
//            packet.put("tvalue",trajectory.tValues.get((int)Math.round(Constants.getTime()/100000000.0-startTime)));
//            packet.put("position",trajectory.equation(trajectory.tValues.get((int)Math.round(Constants.getTime()/100000000.0-startTime))));
//            packet.put("velocity",trajectory.velocities(trajectory.tValues.get((int)Math.round(Constants.getTime()/100000000.0-startTime))));
//            packet.put("velocityX",velocityNormalized.getX());
//
//            packet.put("velocityNormalized",velocityNormalized.getY());
            DashboardUtil.drawRobot(packet.fieldOverlay(), new Pose2d(Constants.robotPose.getX() * .0394, Constants.robotPose.getY() * .0394, (Constants.robotPose.getHeading())));
            packet.fieldOverlay().strokePolyline(xvals, yvals);
            ftcDashboard.sendTelemetryPacket(packet);


        }
    }
}
