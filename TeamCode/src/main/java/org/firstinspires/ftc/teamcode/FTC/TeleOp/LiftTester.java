package org.firstinspires.ftc.teamcode.FTC.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.FTC.Commands.GoToHeight;
import org.firstinspires.ftc.teamcode.FTC.Commands.UpdateLift;
import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

@TeleOp
public class LiftTester extends CommandOpMode {
    LoggerTool telemetry1;
    private LiftSubsystem lift;
    private int newLevel = 0;

    @Override
    public void initialize() {
        telemetry1 = new LoggerTool(telemetry);
        CommandScheduler.getInstance().reset();

        CustomLocalization l = new CustomLocalization(new Pose2d(300, -1500, -Math.PI / 2.0), hardwareMap);

                      lift = new LiftSubsystem();                   register(lift);
        ClawSubsystem claw = new ClawSubsystem();                   register(claw);
        IntakeSubsystem intake = new IntakeSubsystem(telemetry1);   register(intake);
        DriveSubsystem drive = new DriveSubsystem(l, telemetry1);   register(drive);
        Robot.robotInit(hardwareMap, l, telemetry1, intake, claw);
        Robot.liftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        GamepadEx pad2 = new GamepadEx(gamepad2);

        pad2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
            new InstantCommand(() -> {
                newLevel = Robot.level - 1;
                if (newLevel < 0) newLevel = LiftSubsystem.rowHeights.length - 1;
                else if (newLevel > LiftSubsystem.rowHeights.length - 1) newLevel = 0;

                schedule(new GoToHeight(lift, claw, newLevel));
            })
        );

        pad2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
            new InstantCommand(() -> {
                newLevel = Robot.level + 1;
                if (newLevel < 0) newLevel = LiftSubsystem.rowHeights.length - 1;
                else if (newLevel > LiftSubsystem.rowHeights.length - 1) newLevel = 0;

                schedule(new GoToHeight(lift, claw, newLevel));
            })
        );

        pad2.getGamepadButton(GamepadKeys.Button.A).whenPressed(
            new InstantCommand(() -> {
                claw.update(ClawSubsystem.ClawState.CLOSED);
            })
        );
    }

    @Override
    public void run() {
        Robot.l.update();
        CommandScheduler.getInstance().run();

        Robot.telemetry.add("robot lift level", Robot.level);
        Robot.telemetry.add("liftSubsystem lift level", LiftSubsystem.index1);
        Robot.telemetry.add("encoder", Robot.liftEncoder.getCurrentPosition());
        Robot.telemetry.add("Arm", Robot.arm1.getPosition());
        Robot.telemetry.add("Wrist", Robot.wrist1.getPosition());
        Robot.telemetry.update();
    }

}
