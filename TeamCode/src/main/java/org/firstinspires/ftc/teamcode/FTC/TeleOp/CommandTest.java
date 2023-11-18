package org.firstinspires.ftc.teamcode.FTC.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FTC.Commands.DriveToBackdrop;
import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.DriveSubsystem;

import java.util.logging.Logger;

@TeleOp
public class CommandTest extends CommandOpMode {
    CustomLocalization l;
    LoggerTool telemetry;
    DriveSubsystem driveSubsystem;
    GamepadEx gamepadEx1;

    @Override
    public void initialize() {
        gamepadEx1 = new GamepadEx(gamepad1);
        CommandScheduler.getInstance().reset();
        l = new CustomLocalization(new Pose2d(-1500, 300, 0), hardwareMap);
        telemetry = new LoggerTool();
        driveSubsystem = new DriveSubsystem(l);
        register(driveSubsystem);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> schedule(new DriveToBackdrop()));

    }

}
