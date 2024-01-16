package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.MultipleTrajectoryRunner;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.Trajectory;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.TrajectoryRunner;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

import java.util.ArrayList;

@Config
public class DriveToStackRedTruss extends CommandBase {
    TrajectoryRunner tr, tr1;
    MultipleTrajectoryRunner mtr;
    boolean first = false;
    Boolean finished = false;
    public static double leftRed = 0.0;
    public static double stackClose = 100.0;

    @Override
    public void initialize() {
        Trajectory t = new Trajectory(new Pose2d(Constants.robotPose.getY() * -1.0, Constants.robotPose.getX()), new Pose2d(1400 - leftRed, 300), new Pose2d(0, -1200), new Pose2d(0, -1200), new Pose2d(0, 0), new Pose2d(0, 0), true, false);
        Trajectory t1 = new Trajectory(new Pose2d(1400, 300), new Pose2d(1400 - leftRed, -900), new Pose2d(0, -00), new Pose2d(0, -00), new Pose2d(0, 0), new Pose2d(0, 0), false, false);
        Trajectory t2 = new Trajectory(new Pose2d(1400, -900), new Pose2d(800, -1500 - stackClose), new Pose2d(0, -1800), new Pose2d(0, -00), new Pose2d(0, 0), new Pose2d(0, 0), false, false);
        Trajectory t3 = new Trajectory(new Pose2d(800, -1500 - stackClose), new Pose2d(900, 900), new Pose2d(-2270, 00), new Pose2d(2490, -00), new Pose2d(0, 0), new Pose2d(0, 0), false, false);
        Trajectory t4 = new Trajectory(new Pose2d(900, 900), new Pose2d(864, 1145), new Pose2d(0, 00), new Pose2d(0, -00), new Pose2d(0, 0), new Pose2d(0, 0), false, true);

        tr = new TrajectoryRunner(Robot.hardwareMap, Robot.customLocalization, t, 180.0, TrajectoryRunner.HeadingType.ConstantHeadingVelo, Robot.telemetry);
        tr1 = new TrajectoryRunner(Robot.hardwareMap, Robot.customLocalization, t1, 180.0, TrajectoryRunner.HeadingType.ConstantHeadingVelo, Robot.telemetry);
        TrajectoryRunner tr2 = new TrajectoryRunner(Robot.hardwareMap, Robot.customLocalization, t2, 180.0, TrajectoryRunner.HeadingType.ConstantHeadingVelo, Robot.telemetry);
        TrajectoryRunner tr3 = new TrajectoryRunner(Robot.hardwareMap, Robot.customLocalization, t3, 180.0, TrajectoryRunner.HeadingType.ConstantHeadingVelo, Robot.telemetry);
        TrajectoryRunner tr4 = new TrajectoryRunner(Robot.hardwareMap, Robot.customLocalization, t4, 180.0, TrajectoryRunner.HeadingType.ConstantHeadingVelo, Robot.telemetry);

        ArrayList<TrajectoryRunner> trs = new ArrayList<>();
        trs.add(tr);
        trs.add(tr1);
        trs.add(tr2);
        trs.add(tr3);
        trs.add(tr4);
        mtr = new MultipleTrajectoryRunner(trs);
        mtr.start();
    }

    @Override
    public void execute() {

        mtr.update();
        if (mtr.currentT == 4 && !first) {
            CommandScheduler.getInstance().schedule(new GoToHeight(Robot.liftSubsystem, Robot.clawSubsystem, 4));
            first = true;
        }
        if (mtr.finished) {
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return (finished);
    }
}
