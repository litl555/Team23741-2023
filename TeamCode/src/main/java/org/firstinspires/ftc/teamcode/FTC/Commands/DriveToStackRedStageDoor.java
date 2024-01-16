package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.FTC.PathFollowing.Trajectory;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.TrajectoryRunner;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

public class DriveToStackRedStageDoor extends CommandBase {
    TrajectoryRunner tr, tr1;
    int currentT = 0;
    boolean finished = false;

    @Override
    public void initialize() {
        Trajectory t = new Trajectory(new Pose2d(800, 1145), new Pose2d(250, -1500), new Pose2d(-2000, -0), new Pose2d(0, -00), new Pose2d(0, 0), new Pose2d(0, 0), true, true);
        Trajectory t1 = new Trajectory(new Pose2d(250, -1500), new Pose2d(800, 1145), new Pose2d(0, 0), new Pose2d(2000, -0), new Pose2d(0, 0), new Pose2d(0, 0), true, true);
        tr = new TrajectoryRunner(Robot.hardwareMap, Robot.customLocalization, t, 180.0, TrajectoryRunner.HeadingType.ConstantHeadingVelo, Robot.telemetry);
        tr1 = new TrajectoryRunner(Robot.hardwareMap, Robot.customLocalization, t1, 180.0, TrajectoryRunner.HeadingType.ConstantHeadingVelo, Robot.telemetry);
        tr.start();
    }

    @Override
    public void execute() {
        if (currentT == 0) {
            tr.update();
        }
        if (currentT == 1) {
            tr1.update();
        }
        if (tr.currentState == TrajectoryRunner.State.FINISHED) {
            currentT = 1;
            tr1.start();
            tr.currentState = TrajectoryRunner.State.PRESTART;
        }
        if (tr1.currentState == TrajectoryRunner.State.FINISHED) {
            finished = true;
        }

    }

    @Override
    public boolean isFinished() {
        return (finished);
    }
}
