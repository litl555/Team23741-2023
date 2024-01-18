package org.firstinspires.ftc.teamcode.FTC.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.Trajectory;

public class TestTrajVisualizer extends LinearOpMode {
    LoggerTool log;

    // robot coordinates
    public static double init_x = 0, init_y = 0;

    // field coordinates
    public static double pos_s_x = 0, pos_s_y = 0;
    public static double pos_e_x = 0, pos_e_y = 0;
    public static double v_s_x = 0, v_s_y;
    public static double v_e_x = 0, v_e_y;
    public static double pos_x = 0, pos_y = 0;
    public static boolean graphTrajectory = true, graphPosition = true;

    @Override
    public void runOpMode() {
        log = new LoggerTool(telemetry);
        waitForStart();

        CustomLocalization loc = new CustomLocalization(new Pose2d(init_x, init_y, -Math.PI / 2.0), hardwareMap);

        while (opModeIsActive() && !isStopRequested()) {
            Pose2d pos_s = new Pose2d(pos_s_x, pos_s_y);
            Pose2d pos_e = new Pose2d(pos_e_x, pos_e_y);

            Pose2d v_s = new Pose2d(v_s_x, v_s_y);
            Pose2d v_e = new Pose2d(v_e_x, v_e_y);

            Pose2d pos_single = new Pose2d(pos_x, pos_y);

            if (graphTrajectory) log.setCurrentTrajectory(new Trajectory(pos_s, pos_e, v_s, v_e, new Pose2d(0, 0), new Pose2d(0, 0), true, true));
            else log.setCurrentTrajectoryNull();

            if (graphPosition) log.drawPoint(pos_single);

            log.update();
        }
    }
}
