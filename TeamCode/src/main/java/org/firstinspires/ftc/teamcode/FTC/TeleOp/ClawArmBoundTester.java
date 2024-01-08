package org.firstinspires.ftc.teamcode.FTC.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.FTC.Commands.GoToHeight;
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
    double speed = .01;
    double armPos = 0.5;
    double clawPos = 0.5;

    private int newLevel = 0;
    private LiftSubsystem lift;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        LoggerTool tele = new LoggerTool(telemetry);
        gamepad = new GamepadEx(gamepad1);
        lift = new LiftSubsystem();
        ClawSubsystem claw = new ClawSubsystem();
        IntakeSubsystem intake = new IntakeSubsystem(tele);
        CustomLocalization l = new CustomLocalization(new Pose2d(0, 0, 0), hardwareMap);
        DriveSubsystem drive = new DriveSubsystem(l, tele);
        Robot.robotInit(hardwareMap, l, tele, intake, claw);

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(new InstantCommand(() -> armPos += speed).andThen(new InstantCommand(() -> claw.setArm(armPos))));
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(new InstantCommand(() -> armPos -= speed).andThen(new InstantCommand(() -> claw.setArm(armPos))));
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whileHeld(new InstantCommand(() -> clawPos += speed).andThen(new InstantCommand(() -> claw.setWrist(clawPos))));
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whileHeld(new InstantCommand(() -> clawPos -= speed).andThen(new InstantCommand(() -> claw.setWrist(clawPos))));

        gamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> {
            claw.setWrist(0.5);
            claw.setArm(0.5);
        }));

        gamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(() -> {
            claw.setWrist(ClawSubsystem.zero.wrist);
            claw.setArm(ClawSubsystem.zero.wrist);
        }));

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(() -> speed += 0.0005));
        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new InstantCommand(() -> speed -= 0.0005));

        gamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand(() -> {
                    newLevel = Robot.level - 1;
                    if (newLevel < 0) newLevel = LiftSubsystem.rowHeights.length - 1;
                    else if (newLevel > LiftSubsystem.rowHeights.length - 1) newLevel = 0;

                    schedule(new GoToHeight(lift, claw, newLevel));
                })
        );

        gamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand(() -> {
                    newLevel = Robot.level + 1;
                    if (newLevel < 0) newLevel = LiftSubsystem.rowHeights.length - 1;
                    else if (newLevel > LiftSubsystem.rowHeights.length - 1) newLevel = 0;

                    schedule(new GoToHeight(lift, claw, newLevel));
                })
        );

        GamepadEx pad2 = new GamepadEx(gamepad2);

        pad2.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> claw.update(ClawSubsystem.ClawState.CLOSED)));
        pad2.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(() -> claw.update(ClawSubsystem.ClawState.OPEN)));
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        Robot.telemetry.add("Arm1", Robot.arm1.getPosition());
        Robot.telemetry.add("Arm2", Robot.arm2.getPosition());
        Robot.telemetry.add("Wrist1", Robot.wrist1.getPosition());
        Robot.telemetry.add("Wrist2", Robot.wrist2.getPosition());

        Robot.telemetry.add("delta Arm1", Robot.arm1.getPosition() - ClawSubsystem.zero.arm);
        Robot.telemetry.add("delta Arm2", Robot.arm2.getPosition() - ClawSubsystem.zero.arm);
        Robot.telemetry.add("delta Wrist1", Robot.wrist1.getPosition() - ClawSubsystem.zero.wrist);
        Robot.telemetry.add("delta Wrist2", Robot.wrist2.getPosition() - ClawSubsystem.zero.wrist);
        Robot.telemetry.add("Speed", speed);

        Robot.telemetry.add("lift level", lift.read());
        Robot.telemetry.update();

    }
}
