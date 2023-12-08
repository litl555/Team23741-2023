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
public class SafeTeleop extends CommandOpMode {
    int levelCount = 0;
    private boolean isDroppingFirst = true, isHalfClosed = true, isControllingWrist = true, isLiftOverride = false, isClawInTray = false;
    private int clawMode = 0;
    private LoggerTool telemetry1;

    private ClawSubsystem claw;
    private boolean turning=false;
    @Override
    public void initialize() {
        // initialization
        telemetry1 = new LoggerTool(telemetry);
        CommandScheduler.getInstance().reset();

        CustomLocalization l = new CustomLocalization(new Pose2d(300, -1500, -Math.PI / 2.0), hardwareMap);

        LiftSubsystem lift = new LiftSubsystem(); //register(lift);
        claw = new ClawSubsystem(); register(claw);
        IntakeSubsystem intake = new IntakeSubsystem(telemetry1); register(intake);
        DriveSubsystem drive = new DriveSubsystem(l, telemetry1); register(drive);

        // liam will not notice this and this will cause bugs - with love, leo
        //What in the world is that (insert sobbing emoji here)
        assert claw.rowArm.size() == claw.rowWrist.size();

        GamepadEx pad1 = new GamepadEx(gamepad1);
        GamepadEx pad2 = new GamepadEx(gamepad2);

        // lift controls
        pad2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whileHeld(new InstantCommand(() -> lift.setPower(TeleOpConstants.liftUpSpeed)))
                .whenReleased(new InstantCommand(() -> lift.setPower(0.0)));

        pad2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whileHeld(new InstantCommand(() -> lift.setPower(-1 * TeleOpConstants.liftDownSpeed)))
                .whenReleased(new InstantCommand(() -> lift.setPower(0.0)));
        pad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new InstantCommand(() -> lift.setPower(TeleOpConstants.liftUpSpeed)))
                .whenReleased(new InstantCommand(() -> lift.setPower(0.0)));

        pad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new InstantCommand(() -> lift.setPower(-1 * TeleOpConstants.liftDownSpeed)))
                .whenReleased(new InstantCommand(() -> lift.setPower(0.0)));

        // claw controls
        pad2.getGamepadButton(GamepadKeys.Button.Y) // open claw
                .whenPressed(
                        new ConditionalCommand(
                                new InstantCommand(() -> claw.update(ClawSubsystem.ClawState.OPENONE)),
                                new SequentialCommandGroup( // on false
                                        new InstantCommand(() -> claw.setWrist(TeleOpConstants.wristPlacing + 0.012)),
                                        new WaitCommand(200),
                                        new InstantCommand(() -> claw.update(ClawSubsystem.ClawState.OPEN))),
                                () -> { // condition
                                    isDroppingFirst = !isDroppingFirst;
                                    return !isDroppingFirst; // since we have to invert it first
                                }));

        pad2.getGamepadButton(GamepadKeys.Button.X) // close claw
                .whenPressed(
                        new ConditionalCommand(
                            new InstantCommand(() -> {
                                isDroppingFirst = true;
                                if (isHalfClosed) claw.update(ClawSubsystem.ClawState.CLOSED);
                                else claw.update(ClawSubsystem.ClawState.HALFCLOSE);

                                isHalfClosed = !isHalfClosed;
                            }),
                            new SequentialCommandGroup(
                                    new InstantCommand(() -> isDroppingFirst = true),
                                    new InstantCommand(() -> claw.setWrist(TeleOpConstants.wristIntakeGrab)),
                                    new WaitCommand(350),
                                    new InstantCommand(() -> claw.update(ClawSubsystem.ClawState.CLOSED)),
                                    new WaitCommand(1500),
                                    new InstantCommand(() -> lift.setPower(0.2)),
                                    new WaitCommand(TeleOpConstants.liftWait1),
                                    new InstantCommand(() -> claw.setArm(TeleOpConstants.wristIntakeGrab + TeleOpConstants.armAdjust1)),
                                    new WaitCommand(TeleOpConstants.armWait2),
                                    new InstantCommand(() -> claw.setArm(TeleOpConstants.wristIntakeGrab + TeleOpConstants.armAdjust1 + TeleOpConstants.armAdjust2)),
                                    new WaitCommand(TeleOpConstants.armWait3),
                                    new InstantCommand(() -> lift.setPower(0))

                            ),
                            () -> isLiftOverride
                        ));

        // wrist controls
        pad2.getGamepadButton(GamepadKeys.Button.B) // go to current mode
                .whenPressed(new InstantCommand(() -> claw.syncRows(clawMode)));

        // arm controls
        pad2.getGamepadButton(GamepadKeys.Button.A) // go to intake position
                .whenPressed(new InstantCommand(() -> {
                    clawMode = 0;
                    claw.syncRows(0);

                    if (isClawInTray) {
                        claw.update(ClawSubsystem.ClawState.OPEN); // does this need to lift up lift?
                    } else {
                        isHalfClosed = false;
                        claw.update(ClawSubsystem.ClawState.CLOSED);
                    }

                    isClawInTray = !isClawInTray;
                }));

        // arm and wrist controls -> bumpers are used to control level
        // if we are in override bumpers control rotation of whatever were controlling
        pad2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER) // claw mode += 1
                .whenPressed(new InstantCommand(() -> {
                    if (!isLiftOverride) setClawMode(1);
                })) // conditional commands are fake
                .whileHeld(new InstantCommand(() -> { // if we are in override then holding bumpers should continuously cause change
                    if (isLiftOverride) clawOverride(1);
                }));

        pad2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER) // claw mode -= 1
                .whenPressed(new InstantCommand(() -> {
                    if (!isLiftOverride) setClawMode(-1);
                }))
                .whileHeld(new InstantCommand(() -> {
                    if (isLiftOverride) clawOverride(-1);
                }));

        pad2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT) // change if we are wrist or arm
                .whenPressed(new InstantCommand(() -> isControllingWrist = !isControllingWrist));

        // overrides
        pad2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(() -> isLiftOverride = !isLiftOverride));

        // intake controls
        schedule(new RunCommand(() -> {
            double rt = pad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            double lt = pad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

            if (rt != 0.0 || lt != 0.0) intake.setPower(lt - rt);
            else if (Robot.intakeMotor.getPower() != 0) intake.setPower(0);
        }));

        /*

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
        */

        Robot.robotInit(hardwareMap, l, telemetry1, intake, claw);
        drive.setDefaultCommand(new Drive(drive, gamepad1));
    }

    private void clawOverride(double mult) {
        if (isControllingWrist) claw.updateWrist(mult * TeleOpConstants.overrideWristSpeed);
        else claw.updateArm(mult * TeleOpConstants.overrideArmSpeed);
    }

    private void setClawMode(int change) {
        clawMode += change;

        if (clawMode < 0) clawMode = claw.rowWrist.size() - 1;
        else if (clawMode > claw.rowWrist.size() - 1) clawMode = 0;
    }

    @Override
    public void run() {
        Robot.l.update();

        CommandScheduler.getInstance().run();

        telemetry1.add("OVERRIDE", isLiftOverride);
        telemetry1.add("CONTROLLING", isControllingWrist ? "wrist" : "arm");
        telemetry1.add("CLAW IN TRAY", isClawInTray);
        telemetry1.add("===", "===");
        telemetry1.add("MODE", clawMode);
        telemetry1.add("======", "======");
        telemetry1.add("ARM ROTATION", Robot.arm1.getPosition());
        telemetry1.add("WRIST ROTATION", Robot.wrist1.getPosition());
        telemetry1.add("angle", Constants.angle);
        telemetry1.add("pose", Constants.robotPose);
        Robot.telemetry.update();
    }
}
