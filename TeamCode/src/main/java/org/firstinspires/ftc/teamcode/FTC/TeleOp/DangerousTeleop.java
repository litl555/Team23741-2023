package org.firstinspires.ftc.teamcode.FTC.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
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
import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
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
public class DangerousTeleop extends CommandOpMode {
    private int liftLevel = 0;

    @Override
    public void initialize() {
        // initialization
        LoggerTool telemetry1 = new LoggerTool(telemetry);
        CommandScheduler.getInstance().reset();

        CustomLocalization l = new CustomLocalization(new Pose2d(300, -1500, -Math.PI / 2.0), hardwareMap);

        LiftSubsystem lift = new LiftSubsystem();                   register(lift);
        ClawSubsystem claw = new ClawSubsystem();                   register(claw);
        IntakeSubsystem intake = new IntakeSubsystem(telemetry1);   register(intake);
        DriveSubsystem drive = new DriveSubsystem(l, telemetry1);   register(drive);

        GamepadEx pad1 = new GamepadEx(gamepad1);
        GamepadEx pad2 = new GamepadEx(gamepad2);


        pad2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
            new InstantCommand(() -> {
                liftLevel++;
                if (liftLevel > LiftSubsystem.rowHeights.length - 1) liftLevel = 0;
            })
        );

        pad2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
            new InstantCommand(() -> {
                liftLevel--;
                if (liftLevel < 0) liftLevel = LiftSubsystem.rowHeights.length - 1;
            })
        );

        pad2.getGamepadButton(GamepadKeys.Button.Y).whenPressed( // go to current lift level
            new InstantCommand(() -> {
                if (liftLevel == Robot.level) return;

                // if entering tray make sure to prepare to pick up pixel
                if (liftLevel == 0 && Robot.level != 1) {
                    schedule(new SequentialCommandGroup(
                        new GoToHeight(lift, claw, 1),
                        new WaitCommand(200),
                        new GoToHeight(lift, claw, 0)
                    ));
                } else schedule(new GoToHeight(lift, claw, liftLevel));
            })
        );

        pad2.getGamepadButton(GamepadKeys.Button.X).whenPressed( // go to tray
            new InstantCommand(() -> {
                liftLevel = 0;
                if (Robot.level == 0) return;
                else if (Robot.level == 1) schedule(new GoToHeight(lift, claw, 0));
                else schedule(new SequentialCommandGroup(
                    new GoToHeight(lift, claw, 1),
                    new WaitCommand(200),
                    new GoToHeight(lift, claw, 0)
                ));
            })
        );

        pad2.getGamepadButton(GamepadKeys.Button.B).whenPressed( // toggle open
            new InstantCommand(() -> {
                if (claw.currentState == ClawSubsystem.ClawState.OPENONE) claw.update(ClawSubsystem.ClawState.OPEN);
                else claw.update(ClawSubsystem.ClawState.OPENONE);
            })
        );

        pad2.getGamepadButton(GamepadKeys.Button.A).whenPressed(
            new InstantCommand(() -> claw.update(ClawSubsystem.ClawState.CLOSED))
        );

        pad2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
            new InstantCommand(() -> liftLevel = 3)
        );
        
        // intake controls
        schedule(new RunCommand(() -> {
            double rt = pad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            double lt = pad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

            if (rt != 0.0 || lt != 0.0) intake.setPower(lt - rt);
            else if (Robot.intakeMotor.getPower() != 0) intake.setPower(0);
        }));

        Robot.robotInit(hardwareMap, l, telemetry1, intake, claw);
        drive.setDefaultCommand(new Drive(drive, gamepad1));
    }

    @Override
    public void run() {
        Robot.telemetry.add("CURRENT LIFT LEVEL", liftLevel);
        Robot.telemetry.add("ROBOT LEVEL", Robot.level);

        Robot.l.update();
        CommandScheduler.getInstance().run();
        Robot.telemetry.update();
    }
}
