package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.FTC.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

public class Drive extends CommandBase {
    Gamepad gamepad;
    DriveSubsystem drive;

    public Drive(DriveSubsystem drive, Gamepad gamepad) {
        this.gamepad = gamepad;
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (!Robot.isBusy) {
            if (gamepad.left_bumper || gamepad.right_bumper) {
                if (gamepad.right_bumper) drive.l.setMotorPowers(0.5, -0.5, -0.5, 0.5);
                else if (gamepad.left_bumper) drive.l.setMotorPowers(-0.5, 0.5, 0.5, -0.5);
                //if (gamepad.left_bumper) drive.l.setWeightedDrivePowers(new Pose2d(1, 0, 0));
                //else if (gamepad.right_bumper) drive.l.setWeightedDrivePowers(new Pose2d(-1, 0, 0));
            } else {
                if (Robot.forwardIsForward) drive.l.setWeightedDrivePowers(new Pose2d(gamepad.left_stick_x, gamepad.left_stick_y, Math.signum(gamepad.right_stick_x) * Math.pow(gamepad.right_stick_x, 4) / 3.0 * 2.0));
                else drive.l.setWeightedDrivePowers(new Pose2d(-gamepad.left_stick_x, -gamepad.left_stick_y, Math.signum(gamepad.right_stick_x) * Math.pow(gamepad.right_stick_x, 4) / 3.0 * 2.0));
            }
        }
    }
}
