package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.Line;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.MultipleTrajectoryRunner;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.Trajectory;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.TrajectoryRunner;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

import java.util.ArrayList;

///Drives to central position of backdrop
public class DriveToBackdrop extends CommandBase {
    private boolean isFinished = false;
    Trajectory tr = new Trajectory(new Pose2d(-Constants.robotPose.getY(), Constants.robotPose.getX()), new Pose2d(900, 900), new Pose2d(4040, 150), new Pose2d(1800, 000), new Pose2d(0, 0), new Pose2d(0, 0), true, true);
    //    Trajectory tr1 = new Trajectory(new Pose2d(900, 600), new Pose2d(900, 600), new Pose2d(0, 800), new Pose2d(0, 500), new Pose2d(0, 0), new Pose2d(0, 0), true, false);
    //Line line = new Line(new Pose2d(900, -900), new Pose2d(901, 1000), false, true);
    MultipleTrajectoryRunner mtr;
    Pose2d backdropPos = new Pose2d(2700, 2400);
//    Trajectory tr2 = new Trajectory(new Pose2d(900, 1750), backdropPos, new Pose2d(1490, 950), new Pose2d(950, 1350), new Pose2d(-160, 270), new Pose2d(0, 0), false, true);

    HardwareMap hardwareMap = Robot.hardwareMap;
    LoggerTool telemetry = Robot.telemetry;
    CustomLocalization l;

    public DriveToBackdrop(DriveSubsystem robot, LoggerTool telemetry) {
        this.l = robot.l;

        ArrayList<TrajectoryRunner> trajectoryRunners = new ArrayList<>();


        trajectoryRunners.add(new TrajectoryRunner(hardwareMap, l, tr, 0, TrajectoryRunner.HeadingType.ConstantHeadingVelo, telemetry));

        //trajectoryRunners.add(new TrajectoryRunner(hardwareMap, l, line, 0, TrajectoryRunner.HeadingType.ConstantHeadingVelo, telemetry));
//        trajectoryRunners.add(new TrajectoryRunner(hardwareMap, l, tr2, 0, TrajectoryRunner.HeadingType.ConstantHeadingVelo, telemetry));
        mtr = new MultipleTrajectoryRunner(trajectoryRunners);
    }

    @Override
    public void initialize() {
        mtr.start();
    }

    //Execute will be called each loop until isFinished=true
    @Override
    public void execute() {


        mtr.update();
        //telemetry.update();
        if (mtr.finished) {
            isFinished = true;
        }
    }

    public void end() {
        Robot.isBusy = false;
    }

    @Override
    public boolean isFinished() {
        return (isFinished);

    }
}
