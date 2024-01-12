package org.firstinspires.ftc.teamcode.FTC.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.FTC.Commands.UpdateLift;
import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

public class LiftTesting extends CommandOpMode {
    private int levelCount = 0;

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
        Robot.robotInit(hardwareMap, l, telemetry1, intake, claw);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);
        gamepadEx2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new ParallelCommandGroup(
                new InstantCommand(() -> levelCount--),
                new UpdateLift(lift)
        ));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new ParallelCommandGroup(

                new UpdateLift(lift)
        ));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new ParallelCommandGroup(
                new InstantCommand(() -> levelCount++),
                new UpdateLift(lift)
        ));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new UpdateLift(lift));
    }
}
