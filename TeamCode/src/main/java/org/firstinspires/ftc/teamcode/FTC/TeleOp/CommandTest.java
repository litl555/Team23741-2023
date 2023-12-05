package org.firstinspires.ftc.teamcode.FTC.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FTC.Commands.Drive;
import org.firstinspires.ftc.teamcode.FTC.Commands.DriveToBackdrop;
import org.firstinspires.ftc.teamcode.FTC.Commands.GetUpdatedPoseXHeading;
import org.firstinspires.ftc.teamcode.FTC.Commands.UpdateIntake;
import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.logging.Logger;

@TeleOp
public class CommandTest extends CommandOpMode {
    CustomLocalization l;
    LoggerTool telemetry1;
    DriveSubsystem driveSubsystem;
    GamepadEx gamepadEx1;
    Robot robot;
    SampleMecanumDrive dr;
    @Override
    public void initialize() {
        telemetry1 = new LoggerTool(telemetry);
        Constants.angle = 0;
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(telemetry1);
        LiftSubsystem lift = new LiftSubsystem();
        ClawSubsystem claw = new ClawSubsystem();
        Robot.isBusy = false;
        Robot.robotInit(hardwareMap, l, telemetry1, intakeSubsystem, claw);
        gamepadEx1 = new GamepadEx(gamepad1);
        CommandScheduler.getInstance().reset();
        l = new CustomLocalization(new Pose2d(-1500, 1500, 0), hardwareMap);

        driveSubsystem = new DriveSubsystem(l, telemetry1);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> schedule(new GetUpdatedPoseXHeading(l)));
        register(driveSubsystem);
        register(intakeSubsystem);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(new UpdateIntake(intakeSubsystem, IntakeSubsystem.IntakePowerSetting.INTAKE)).whenReleased(new UpdateIntake(intakeSubsystem, IntakeSubsystem.IntakePowerSetting.IDLE));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(new UpdateIntake(intakeSubsystem, IntakeSubsystem.IntakePowerSetting.OUTTAKE)).whenReleased(new UpdateIntake(intakeSubsystem, IntakeSubsystem.IntakePowerSetting.IDLE));

//        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.IntakePowerSetting.INTAKE),intakeSubsystem)).whenReleased(new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.IntakePowerSetting.IDLE),intakeSubsystem));
//        intakeSubsystem.setDefaultCommand(new RunCommand(()->intakeSubsystem.update(IntakeSubsystem.IntakePowerSetting.IDLE),intakeSubsystem));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> schedule(new DriveToBackdrop(driveSubsystem, telemetry1).interruptOn(() -> gamepad1.b).whenFinished(() -> Robot.isBusy = false)));
        driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, gamepad1));

    }

    @Override
    public void run() {
//        dr.updatePoseEstimate();
        l.update();
        Robot.telemetry.add("angle", Constants.angle);
//        Constants.robotPose=new Pose2d(dr.getPoseEstimate().getX()*(double)25.4-(double)1500.0,dr.getPoseEstimate().getY()*(double)25.4+(double)1500.0,dr.getPoseEstimate().getHeading());

//        try {
//            Constants.velocity = dr.getPoseVelocity().times(25.4);
//        }
//        catch (Exception e){
//            Constants.velocity=new Pose2d(0,0,0);
//        }

        CommandScheduler.getInstance().run();
        telemetry.update();
        //l.update();
    }

}
