package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.FTC.PathFollowing.ActualMultiTrajRunner;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.MultiTrajEvent;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.SimpleTrajectory;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.TrajectoryRunner;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

public class IntakePixelFromStack extends CommandBase {
    // assumptions:
    // we are directly in front of the stack, intake facing the stack

    private final double maxDistanceFromWall = 150;
    private final double robotLengthWithIntake = 567; // mm

    private int numToCollect, collected, initialPixelCount;
    private double maxTime, robotHeading, startTime;
    private TrajectoryRunner runner;
    private boolean finished = false;

    // time in milliseconds
    public IntakePixelFromStack(int numToCollect, double maxTime, double robotHeading) {
        this.numToCollect = numToCollect;
        this.maxTime = maxTime;
        this.robotHeading = robotHeading;
    }

    @Override
    public void initialize() {
        // idk if customLocal returns radians or degrees and i cant be bothered so just pass in the current heading
        Pose2d startPose = new Pose2d(Robot.customLocalization.getPoseEstimate().getY() * -1.0, Robot.customLocalization.getPoseEstimate().getX());
        Pose2d endPose = new Pose2d(startPose.getX(), -72 * 25.4 + maxDistanceFromWall + (robotLengthWithIntake - Robot.length) + Robot.length / 2.0);

        SimpleTrajectory tr = new SimpleTrajectory(startPose, endPose, new Pose2d(0, 0), new Pose2d(0, 0), robotHeading);
        runner = tr.getTrajectoryRunner(tr.getTrajectory(true, true));

        runner.start();
        startTime = System.currentTimeMillis();
        initialPixelCount = Robot.intakeSubsystem.pixelPassCount;
        //TrajectoryRunner.speed = 0.2;

        Robot.intakeSubsystem.setIntakePosition(IntakeSubsystem.IntakePosition.DOWN);
        Robot.intakeSubsystem.setPower(-1);
    }

    @Override
    public void execute() {
        if (finished) return;

        if (runner.currentState != TrajectoryRunner.State.FINISHED) runner.update();
        collected = Robot.intakeSubsystem.pixelPassCount - initialPixelCount;

        finished = collected == numToCollect || System.currentTimeMillis() - startTime >= maxTime;

        if (finished) {
            Robot.intakeSubsystem.setPower(1);
            //TrajectoryRunner.speed = 0.6;
        }
    }

    @Override
    public boolean isFinished() { return finished; }
}
