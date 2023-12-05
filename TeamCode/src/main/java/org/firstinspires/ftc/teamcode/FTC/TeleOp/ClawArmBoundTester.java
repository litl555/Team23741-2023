package org.firstinspires.ftc.teamcode.FTC.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class ClawArmBoundTester extends CommandOpMode {
    GamepadEx gamepad;
    double speed = .001;
    double armPos = 0.5;
    double clawPos = 0.5;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        LoggerTool tele = new LoggerTool(telemetry);
        gamepad = new GamepadEx(gamepad1);
        LiftSubsystem lift = new LiftSubsystem();
        ClawSubsystem claw = new ClawSubsystem();
        IntakeSubsystem intake = new IntakeSubsystem(tele);
        SampleMecanumDrive dr = new SampleMecanumDrive(hardwareMap);
        CustomLocalization l = new CustomLocalization(new Pose2d(0, 0, 0), hardwareMap, dr);
        DriveSubsystem drive = new DriveSubsystem(l, tele);
        Robot.robotInit(hardwareMap, l, tele, intake, claw);

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(new InstantCommand(() -> armPos += speed).andThen(new InstantCommand(() -> claw.setArm(armPos))));
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(new InstantCommand(() -> armPos -= speed).andThen(new InstantCommand(() -> claw.setArm(armPos))));
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whileHeld(new InstantCommand(() -> clawPos += speed).andThen(new InstantCommand(() -> claw.setWrist(clawPos))));
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whileHeld(new InstantCommand(() -> clawPos -= speed).andThen(new InstantCommand(() -> claw.setWrist(clawPos))));
        Robot.claw.setArm(0.5);
        Robot.claw.setWrist(0.5);
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        Robot.telemetry.add("Arm", armPos);
        Robot.telemetry.add("Claw", clawPos);
        Robot.telemetry.update();

    }
}
