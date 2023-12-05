package org.firstinspires.ftc.teamcode.FTC.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.analysis.function.Log;
import org.firstinspires.ftc.teamcode.FTC.Commands.Drive;
import org.firstinspires.ftc.teamcode.FTC.Commands.GoToHeight;
import org.firstinspires.ftc.teamcode.FTC.Commands.IntakePixel;
import org.firstinspires.ftc.teamcode.FTC.Commands.RetractLift;
import org.firstinspires.ftc.teamcode.FTC.Commands.UpdateClaw;
import org.firstinspires.ftc.teamcode.FTC.Commands.UpdateIntake;
import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;
import org.firstinspires.ftc.teamcode.FTC.Pixels.Types.Pose;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.FTC.Subsystems.IntakeSubsystem.IntakePowerSetting.IDLE;
import static org.firstinspires.ftc.teamcode.FTC.Subsystems.IntakeSubsystem.IntakePowerSetting.INTAKE;
import static org.firstinspires.ftc.teamcode.FTC.Subsystems.IntakeSubsystem.IntakePowerSetting.OUTTAKE;

@TeleOp
public class SafeTeleop extends CommandOpMode {
    int levelCount = 0;

    @Override
    public void initialize() {
        LoggerTool telemetry1 = new LoggerTool(telemetry);
        CommandScheduler.getInstance().reset();
        LiftSubsystem lift = new LiftSubsystem();
        ClawSubsystem claw = new ClawSubsystem();
        register(claw);
        //register(lift);

        IntakeSubsystem intake = new IntakeSubsystem(telemetry1);
        register(intake);
        CustomLocalization l = new CustomLocalization(new Pose2d(0, 0, 0), hardwareMap);
        DriveSubsystem drive = new DriveSubsystem(l, telemetry1);
        register(drive);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new ParallelCommandGroup(
                new InstantCommand(() -> levelCount++),
                new GoToHeight(lift, claw, levelCount)
        ));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new GoToHeight(Robot.lift, Robot.claw, levelCount));
//        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(()->{
//            lift.setPower(1.0);
//        })).whenReleased(new InstantCommand(()->{
//            lift.setPower(0.0);
//        }));
//        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(()->{
//            lift.setPower(-1.0);
//        })).whenReleased(new InstantCommand(()->{
//            lift.setPower(0.0);
//        }));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new ParallelCommandGroup(
                new InstantCommand(() -> levelCount--),
                new GoToHeight(lift, claw, levelCount)
        ));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new ParallelCommandGroup(

                new RetractLift()
        ));

//        gamepadEx2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new UpdateIntake(intake,OUTTAKE)).whenReleased(new UpdateIntake(intake,IDLE));
//        gamepadEx2.getGamepadButton(GamepadKeys.Button.A).whenPressed(new UpdateIntake(intake,INTAKE)).whenReleased(new UpdateIntake(intake,IDLE));
        schedule(new RunCommand(() -> {
            if (gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) != 0.0 || gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) != 0.0) {
                intake.setPower(gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
            } else {
                if (Robot.intakeMotor.getPower() != 0.0) {
                    intake.setPower(0.0);
                }
            }
        }));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new UpdateClaw(claw, ClawSubsystem.ClawState.OPEN));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.X).whenPressed(new UpdateClaw(claw, ClawSubsystem.ClawState.CLOSED));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(() -> intake.setIntakePosition(IntakeSubsystem.IntakePosition.DOWN)));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> intake.setIntakePosition(IntakeSubsystem.IntakePosition.UP)));
        //gamepadEx2.getGamepadButton(GamepadKeys.Button.B).whenPressed(()->schedule(new IntakePixel(intake)));
        Robot.robotInit(hardwareMap, l, telemetry1, intake, claw);
        drive.setDefaultCommand(new Drive(drive, gamepad1));

    }

    @Override
    public void run() {

        Robot.l.update();
        CommandScheduler.getInstance().run();

//        Robot.telemetry.update();
    }
}
