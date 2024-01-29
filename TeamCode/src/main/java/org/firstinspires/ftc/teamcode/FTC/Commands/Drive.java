package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

import static org.firstinspires.ftc.teamcode.FTC.PathFollowing.FollowerConstants.kpa;

@Config
public class Drive extends CommandBase {
    Gamepad gamepad;
    public static double angledes = 0.0;
    public static double angleMult = .001;
    DriveSubsystem drive;


    public Drive(DriveSubsystem drive, Gamepad gamepad) {
        this.gamepad = gamepad;
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void execute() {

        if (!Robot.isBusy) {
            if (Robot.pidControl) {
                double h = -3.0 * (-Constants.angle - angledes);
                angledes += angleMult * gamepad.right_stick_x;
                if (Robot.forwardIsForward)
                    drive.l.setWeightedDrivePowers(new Pose2d(gamepad.left_stick_x, gamepad.left_stick_y, h));
                else
                    drive.l.setWeightedDrivePowers(new Pose2d(-gamepad.left_stick_x, -gamepad.left_stick_y, h));
            } else {
                if (gamepad.left_bumper || gamepad.right_bumper) {
                    if (gamepad.right_bumper) drive.l.setMotorPowers(0.5, -0.5, -0.5, 0.5);
                    else if (gamepad.left_bumper) drive.l.setMotorPowers(-0.5, 0.5, 0.5, -0.5);
                } else {
                    if (Robot.forwardIsForward)
                        drive.l.setWeightedDrivePowers(new Pose2d(gamepad.left_stick_x, gamepad.left_stick_y, Math.signum(gamepad.right_stick_x) * Math.pow(gamepad.right_stick_x, 4) / 3.0 * 2.0));
                    else
                        drive.l.setWeightedDrivePowers(new Pose2d(-gamepad.left_stick_x, -gamepad.left_stick_y, Math.signum(gamepad.right_stick_x) * Math.pow(gamepad.right_stick_x, 4) / 3.0 * 2.0));
                }
            }
        }
    }
}
