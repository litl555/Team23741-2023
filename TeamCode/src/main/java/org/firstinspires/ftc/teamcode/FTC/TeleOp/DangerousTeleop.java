package org.firstinspires.ftc.teamcode.FTC.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.FTC.Commands.Drive;
import org.firstinspires.ftc.teamcode.FTC.Commands.GoToHeight;
import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

@TeleOp
public class DangerousTeleop extends CommandOpMode {
    private int liftLevel = 0;

    private LiftSubsystem lift;
    private ClawSubsystem claw;
    private IntakeSubsystem intake;
    private DriveSubsystem drive;

    @Override
    public void initialize() {
        // initialization
        LoggerTool telemetry1 = new LoggerTool(telemetry);
        CommandScheduler.getInstance().reset();

        CustomLocalization l = new CustomLocalization(new Pose2d(300, -1500, -Math.PI / 2.0), hardwareMap);

        lift = new LiftSubsystem();                   register(lift);
        claw = new ClawSubsystem();                   register(claw);
        intake = new IntakeSubsystem(telemetry1);   register(intake);
        drive = new DriveSubsystem(l, telemetry1);   register(drive);

        GamepadEx pad1 = new GamepadEx(gamepad1);
        GamepadEx pad2 = new GamepadEx(gamepad2);

        // GAMEPAD 1 CONTROLS
        {
            drive.setDefaultCommand(new Drive(drive, gamepad1));

            pad1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(() -> Robot.forwardIsForward = true));
            pad1.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> Robot.forwardIsForward = false));
            pad1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(() -> intake.setIntakePosition(IntakeSubsystem.IntakePosition.UP)));
            pad1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(() -> intake.setIntakePosition(IntakeSubsystem.IntakePosition.DOWN)));

            schedule(new RunCommand(() -> {
                double rt = pad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
                double lt = pad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

                if (rt != 0.0 || lt != 0.0) intake.setPower(lt - rt);
                else if (Robot.intakeMotor.getPower() != 0) intake.setPower(0);
            }));
        }

        // GAMEMPAD 2 CONTROLS
        {
            pad2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(new InstantCommand(() -> Robot.drone.setPower(1)));
            pad2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenInactive(new InstantCommand(() -> Robot.drone.setPower(0)));

            pad2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> {
                    liftLevel++;
                    if (liftLevel > lift.rowHeights.length - 1) liftLevel = 0;
                })
            );

            pad2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> {
                    liftLevel--;
                    if (liftLevel < 0) liftLevel = lift.rowHeights.length - 1;
                })
            );

            pad2.getGamepadButton(GamepadKeys.Button.Y).whenPressed( // go to current lift level
                new InstantCommand(() -> {
                    if (liftLevel == Robot.level) return;

                    // if entering tray make sure to prepare to pick up pixel
                    if (liftLevel == 0 && Robot.level != 1) {
                        schedule(new SequentialCommandGroup(
                            new GoToHeight(lift, claw, 1),
                            new WaitCommand(300),
                            new GoToHeight(lift, claw, 0)
                        ));
                    } else schedule(new GoToHeight(lift, claw, liftLevel));

                    // currently we have a preset to go straight to ground level when picking up from tray, so update liftLevel to reflect that change
                    if (Robot.level == 0 && liftLevel == 1) liftLevel = 2;
                })
            );

            pad2.getGamepadButton(GamepadKeys.Button.X).whenPressed( // go to tray
                new InstantCommand(() -> {
                    liftLevel = 0;
                    if (Robot.level == 0) return;
                    else if (Robot.level == 1) schedule(new GoToHeight(lift, claw, 0));
                    else schedule(new SequentialCommandGroup(
                        new GoToHeight(lift, claw, 1),
                        new WaitCommand(1000),
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

            // toggling how strong the lift should be during hang
            pad2.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(new InstantCommand(() -> {
                lift.hangOverride = true;
                claw.enableHang();
            }));
            pad2.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(new InstantCommand(() -> lift.hangOverride = false));

            schedule(new RunCommand(() -> {
                if (!lift.hangOverride) return;

                double y = pad2.getLeftY();
                if (Math.abs(y) > 0.2) lift.setPower(-y);
                else lift.setPower(0);
            }));
        }

        Robot.robotInit(hardwareMap, l, telemetry1, intake, claw, lift);
    }

    @Override
    public void run() {
        Robot.telemetry.add("CURRENT LIFT LEVEL (0 based)", liftLevel);
        Robot.telemetry.add("PIXEL LEVEL (1 based)", liftLevel - 2);
        Robot.telemetry.add("ROBOT LEVEL", Robot.level);
        Robot.telemetry.add("LIFT IS OVERRIDDEN", lift.hangOverride);
        Robot.telemetry.add("LIFT POSITION", lift.read());
        Robot.telemetry.add("LIFT POWER LEFT", Robot.liftLeft.getPower());
        Robot.telemetry.add("LIFT POWER RIGHT", Robot.liftRight.getPower());

        Robot.customLocalization.update();
        CommandScheduler.getInstance().run();
        Robot.telemetry.update();
    }
}
