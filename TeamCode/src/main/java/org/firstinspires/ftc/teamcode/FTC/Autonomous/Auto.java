package org.firstinspires.ftc.teamcode.FTC.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.FTC.Commands.DriveToSpikeStripF;
import org.firstinspires.ftc.teamcode.FTC.Commands.GoToHeight;
import org.firstinspires.ftc.teamcode.FTC.Commands.PlacePixelOnBackboardF;
import org.firstinspires.ftc.teamcode.FTC.Commands.PlacePixelOnSpikeStrip;
import org.firstinspires.ftc.teamcode.FTC.Commands.timing.liftUpTime;
import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;
import org.firstinspires.ftc.teamcode.FTC.Pixels.BoardDetectionPipeline;
import org.firstinspires.ftc.teamcode.FTC.Pixels.Types.Pose;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@TeleOp
public class Auto extends LinearOpMode {
    TeamPropPosition pos;
    public static Pose2d startPos = new Pose2d(300, -1500, 0);

    public static double liftControlSpeed = 0.3;
    public static long liftRiseTime = 1000;
    public static double liftRiseSpeed = 0.3;

    private boolean hasStartedAuto = false;

    // RED BACKBOARD SIDE
    @Override
    public void runOpMode() throws InterruptedException {
        LoggerTool telemetry1 = new LoggerTool(telemetry);
        CommandScheduler.getInstance().reset();
        LiftSubsystem lift = new LiftSubsystem();
        ClawSubsystem claw = new ClawSubsystem();

        IntakeSubsystem intake = new IntakeSubsystem(telemetry1);

        //SampleMecanumDrive dr = new SampleMecanumDrive(hardwareMap);
        CustomLocalization l = new CustomLocalization(startPos, hardwareMap);
        DriveSubsystem drive = new DriveSubsystem(l, telemetry1);

        Robot.robotInit(hardwareMap, l, telemetry1, intake, claw);

        waitForStart();

        GamepadEx pad1 = new GamepadEx(gamepad1);

        /*
        pad1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new liftUpTime(lift, claw, liftRiseTime, liftRiseSpeed));
        pad1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> lift.setPower(liftControlSpeed)))
                .whenReleased(new InstantCommand(() -> lift.setPower(0)));
        pad1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> lift.setPower(-liftControlSpeed)))
                .whenReleased(new InstantCommand(() -> lift.setPower(0)));

        pad1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> claw.update(ClawSubsystem.ClawState.CLOSED)));
        pad1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> claw.update(ClawSubsystem.ClawState.OPEN)));
        */

        CommandScheduler.getInstance().schedule(new DriveToSpikeStripF(TeamPropPosition.middle));

        while (opModeIsActive() && !isStopRequested()) {
            Robot.l.update();
            Robot.telemetry.update();

            CommandScheduler.getInstance().run();
            //lift.periodic();
        }
    }
}
