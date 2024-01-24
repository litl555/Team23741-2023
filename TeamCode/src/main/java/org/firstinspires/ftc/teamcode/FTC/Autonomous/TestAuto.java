package org.firstinspires.ftc.teamcode.FTC.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.FTC.Commands.PlacePixelOnBackboardF;
import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class TestAuto extends CommandOpMode {
    SampleMecanumDrive dr;
    CustomLocalization l;
    LoggerTool telemetry1;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telemetry1 = new LoggerTool(telemetry);

        l = new CustomLocalization(new Pose2d(300, -1500, Math.toRadians(90)), hardwareMap);
        LiftSubsystem lift = new LiftSubsystem();
        ClawSubsystem claw = new ClawSubsystem();
        IntakeSubsystem intake = new IntakeSubsystem(telemetry1);
        Robot.robotInit(hardwareMap, l, telemetry1, intake, claw, lift);
        schedule(new PlacePixelOnBackboardF(TeamPropPosition.left));
    }
}
