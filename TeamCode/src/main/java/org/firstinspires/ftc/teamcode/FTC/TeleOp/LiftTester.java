package org.firstinspires.ftc.teamcode.FTC.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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
    private ClawSubsystem claw;
    private static int level = 0;

    @Override
    public void initialize() {
        telemetry1 = new LoggerTool(telemetry);


        GamepadEx pad2 = new GamepadEx(gamepad2);
        CommandScheduler.getInstance().reset();

        CustomLocalization l = new CustomLocalization(new Pose2d(300, -1500, -Math.PI / 2.0), hardwareMap);

        LiftSubsystem lift = new LiftSubsystem(); //register(lift);
        register(lift);
        claw = new ClawSubsystem();
        register(claw);
        IntakeSubsystem intake = new IntakeSubsystem(telemetry1);
        register(intake);
        DriveSubsystem drive = new DriveSubsystem(l, telemetry1);
        register(drive);
        Robot.robotInit(hardwareMap, l, telemetry1, intake, claw);
        Robot.liftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pad2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new SequentialCommandGroup(new InstantCommand(() -> {
                    Robot.level--;
                }), new UpdateLift(lift))
        );
        pad2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new SequentialCommandGroup(new InstantCommand(() -> {

                    Robot.level++;
                }), new UpdateLift(lift))
        );
    }

    @Override
    public void run() {
        Robot.telemetry.add("level", level);

        CommandScheduler.getInstance().run();

        telemetry1.add("encoder", Robot.liftEncoder.getCurrentPosition());
        telemetry1.update();
    }

}
