package org.firstinspires.ftc.teamcode.localiation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;

@TeleOp
public class NewMotioinProfileTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        int counter = 0;
        //Trajectory trajectory = new Trajectory(new Pose2d(0, 0), new Pose2d(0, 1000), new Pose2d(1150, 2080), new Pose2d(1200, 2600), new Pose2d(403, 850), new Pose2d(-453, -110));
        Trajectory trajectory = new Trajectory(new Pose2d(0, 0), new Pose2d(0, 1000), new Pose2d(3000, 4330), new Pose2d(10, 2745), new Pose2d(3110, 1950), new Pose2d(-1000, -600));

        FtcDashboard dash = FtcDashboard.getInstance();
        waitForStart();
        double startTime = Constants.getTime() / 100000000.0;
        Constants.angle = 0;


        ArrayList<Double> mp = trajectory.generateMotionProfile();
        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket p = new TelemetryPacket();
//            p.put("mp", trajectory.amp);
//            p.put("mp1", mp);
            p.put("xAccels", trajectory.xAccels);


            dash.sendTelemetryPacket(p);


        }
    }
}
