package org.firstinspires.ftc.teamcode.FTC.Autonomous;

import static org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot.liftEncoder;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.FTC.Commands.DriveToBackBoardRed;
import org.firstinspires.ftc.teamcode.FTC.Commands.DriveToParkingRed;
import org.firstinspires.ftc.teamcode.FTC.Commands.DriveToSpikeStripRed;
import org.firstinspires.ftc.teamcode.FTC.Commands.DriveToStackRedStageDoor;
import org.firstinspires.ftc.teamcode.FTC.Commands.DriveToStackRedTruss;
import org.firstinspires.ftc.teamcode.FTC.Commands.GoToHeight;
import org.firstinspires.ftc.teamcode.FTC.Commands.RamBoard;
import org.firstinspires.ftc.teamcode.FTC.Commands.UpdateClaw;
import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.R;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous
public class AutoRed extends LinearOpMode {
    // TODO: implement below
    //public static Pose2d startPos = new Pose2d(609.6 - Robot.width / 2.0, -1828.8 + Robot.length / 2.0, -Math.PI / 2.0);
    // center of tile (field coords): 60 in, 12 in
    // corner (towards backboard, aligning wall): 72 in, 24 in

    public static Pose2d startPos = new Pose2d(300, -1500, -Math.PI / 2.0);

    // RED BACKBOARD SIDE
    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();

        LoggerTool telemetry1 = new LoggerTool(telemetry);
        LiftSubsystem lift = new LiftSubsystem();
        ClawSubsystem claw = new ClawSubsystem();
        IntakeSubsystem intake = new IntakeSubsystem(telemetry1);
        CustomLocalization l = new CustomLocalization(startPos, hardwareMap);
        DriveSubsystem drive = new DriveSubsystem(l, telemetry1);

        Robot.robotInit(hardwareMap, l, telemetry1, intake, claw, lift);

        OpenCvCamera cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "outtake_camera"));
        TeamPropDetectionPipeline pipeline = new TeamPropDetectionPipeline(cam, telemetry1, true);

        telemetry1.add("Initialization", "done");

        waitForStart();

        //Robot.intakeSubsystem.setIntakePosition(IntakeSubsystem.IntakePosition.DOWN);

        // detect team prop
        int totalCount = 0;
        int similarityCount = 0;
        TeamPropPosition last = TeamPropPosition.undefined;
        while (similarityCount < 10) {
            if (last != pipeline.propPos || pipeline.propPos == TeamPropPosition.undefined) similarityCount = 0;
            else similarityCount++;

            last = pipeline.propPos;

            totalCount++;

            if (totalCount > 250) break;
        }

        pipeline.destroy();
        cam.stopStreaming();

        CommandScheduler.getInstance().schedule(
            new SequentialCommandGroup(
                // go to spike strip
                new ParallelCommandGroup(
                    new DriveToSpikeStripRed(last),
                    new GoToHeight(lift, Robot.clawSubsystem, 2)),
                // drop pixel
                new UpdateClaw(Robot.clawSubsystem, ClawSubsystem.ClawState.OPENONE),
                new WaitCommand(250),
                // go to board
                new ParallelCommandGroup(
                    new GoToHeight(lift, Robot.clawSubsystem, 3),
                    new DriveToBackBoardRed(last)),
                // put pixel on board
                new RamBoard(),
                new WaitCommand(250),
                new UpdateClaw(Robot.clawSubsystem, ClawSubsystem.ClawState.OPEN),
                new WaitCommand(250),
                new ParallelCommandGroup(
                    // reset lift
                    new SequentialCommandGroup(
                        new WaitCommand(1_000),
                        new SequentialCommandGroup(
                            new GoToHeight(Robot.liftSubsystem, Robot.clawSubsystem, 1),
                            new WaitCommand(200),
                            new GoToHeight(Robot.liftSubsystem, Robot.clawSubsystem, 0))),
                    new DriveToParkingRed()))
        );

        while (opModeIsActive() && !isStopRequested()) {
            Robot.telemetry.add("Detected prop pos from auto", last);
            Robot.telemetry.add("pose", Constants.robotPose);
            Robot.customLocalization.update();
            Robot.telemetry.update();

            CommandScheduler.getInstance().run();
        }
    }
}
