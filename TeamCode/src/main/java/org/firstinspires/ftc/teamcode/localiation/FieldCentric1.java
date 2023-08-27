package org.firstinspires.ftc.teamcode.localiation;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.FeedforwardEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.localiation.CustomLocalization;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name = "Teleoptest")
@Config
public class FieldCentric1 extends LinearOpMode {
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    //Trajectory trajectory=new Trajectory(new Pose2d(0,0),new Pose2d(0,-8),new Pose2d(10,44),new Pose2d(0,45),new Pose2d(549,-60),new Pose2d(540,60));
    Trajectory trajectory = new Trajectory(new Pose2d(0, 0), new Pose2d(0, 1000), new Pose2d(1560, 2610), new Pose2d(580, 2885), new Pose2d(3110, 1950), new Pose2d(-1280, -1250));

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        double glxp = 0;
        double glyp = 0;
        double grxp = 0;
        double glx = 0;
        double gly = 0;
        double grx = 0;
        long lastTime = 0;
        int counter = 0;
        double timeprev = 0;
        //SampleMecanumDrive driver=new SampleMecanumDrive(hardwareMap);
        CustomLocalization customLocalization = new CustomLocalization(new Pose2d(0, 0, 0), hardwareMap);
        double[] xvals = new double[101];
        double[] yvals = new double[101];

        for (int i = 0; i < 100; i++) {
            Pose2d pos = trajectory.equation((double) i / 100.0);
            yvals[i] = (pos.getX() / 25.4);
            xvals[i] = (pos.getY() / 25.4);
        }
        waitForStart();
        timeprev = Constants.getTime() / 1000000000;
        double startTime = Constants.getTime() / (double) 1000000000;
        Constants.angle = 0;

        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();


            //double y=(double)-1*gamepad1.left_stick_y;
            //double x=(double)-1*gamepad1.left_stick_x;
            //driver.setWeightedDrivePower(new Pose2d((double)-1*x * Math.sin((double)-1*driver.getExternalHeading()) + y * Math.cos((double)-1*driver.getExternalHeading()), x * Math.cos((double)-1*driver.getExternalHeading()) + y * Math.sin((double)-1*driver.getExternalHeading()),(double)-1*gamepad1.right_stick_x));
            //driver.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y,-gamepad1.left_stick_x,-gamepad1.right_stick_x));
            glx = gamepad1.left_stick_x;
            grx= gamepad1.right_stick_x;
            gly=gamepad1.left_stick_y;
            //driver.update();
            if(Math.abs(glx-glxp)>.01||Math.abs(grx-grxp)>.01||Math.abs(gly-glyp)>.01) {
                double x = gamepad1.left_stick_x;
                double y = gamepad1.left_stick_y;
                customLocalization.setWeightedDrivePowers(new Pose2d(Math.cos(Constants.angle) * x - Math.sin(Constants.angle) * y, x * Math.sin(Constants.angle) + y * Math.cos(Constants.angle), gamepad1.right_stick_x));
                glxp = glx;
                glyp = gly;
                grxp = grx;
            }


            customLocalization.updateMethod();
            //driver.update();
            packet.put("arcLength",trajectory.length);
            packet.put("X", Constants.robotPose.getX());
            packet.put("Y", Constants.robotPose.getY());

//            packet.put("Heading", Constants.robotPose.getHeading());
            packet.put("loop time",(Constants.getTime()-lastTime)/1000000);
            packet.put("time",Constants.timed);
            packet.put("currentTime",(double)Constants.getTime()/(double)1000000000-startTime);
            packet.put("arclengthsum",Constants.yes);


            lastTime=Constants.getTime();
            DashboardUtil.drawRobot(packet.fieldOverlay(), new Pose2d(Constants.robotPose.getX() * .0394, Constants.robotPose.getY() * .0394, (Constants.robotPose.getHeading())));
            packet.fieldOverlay().strokePolyline(xvals, yvals);
            dashboard.sendTelemetryPacket(packet);

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();

            }
            if(Math.round(Constants.getTime()/100000000.0)!=Math.round(timeprev)) {
                counter++;
            }
            timeprev=Constants.getTime()/100000000.0;




        }

    }
}
