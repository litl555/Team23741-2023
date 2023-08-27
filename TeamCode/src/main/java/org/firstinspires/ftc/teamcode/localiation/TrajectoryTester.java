package org.firstinspires.ftc.teamcode.localiation;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class TrajectoryTester extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        CustomLocalization localization = new CustomLocalization(Constants.startPose, hardwareMap);
        Trajectory trajectory = new Trajectory(new Pose2d(0, 0), new Pose2d(0, 1000), new Pose2d(1560, 2610), new Pose2d(580, 2885), new Pose2d(3110, 1950), new Pose2d(-1280, -1250));
        TrajectoryRunner tr = new TrajectoryRunner(localization, trajectory, 0);
        waitForStart();
        tr.start();

        while (opModeIsActive() && !isStopRequested()) {
            tr.update();

            localization.updateMethod();
        }
    }
}
