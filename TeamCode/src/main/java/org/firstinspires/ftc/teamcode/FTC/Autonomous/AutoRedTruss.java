package org.firstinspires.ftc.teamcode.FTC.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.FTC.Commands.AutoRed.DriveFromBoardToStackRed;
import org.firstinspires.ftc.teamcode.FTC.Commands.AutoRed.Truss.DriveToBackBoardRedTruss;
import org.firstinspires.ftc.teamcode.FTC.Commands.AutoRed.Truss.DriveToSpikeStripRedTruss;
import org.firstinspires.ftc.teamcode.FTC.Commands.GoToHeight;
import org.firstinspires.ftc.teamcode.FTC.Commands.IntakePixelFromStack;
import org.firstinspires.ftc.teamcode.FTC.Commands.RamBoard;
import org.firstinspires.ftc.teamcode.FTC.Commands.RamIntake;
import org.firstinspires.ftc.teamcode.FTC.Commands.UpdateClaw;
import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.FTC.Threading.WriteThread;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Photon(singleThreadOptimized = false, maximumParallelCommands = 8)
@Autonomous(preselectTeleOp = "DangerousTeleop")
public class AutoRedTruss extends LinearOpMode {
    private static final double inToMm = 25.4;
    private static Pose2d startPos = new Pose2d(-48 * inToMm + Robot.width / 2.0,-72 * inToMm + Robot.length / 2.0, -Math.PI / 2.0);
    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();

        LoggerTool telemetry1 = new LoggerTool(telemetry);
        LiftSubsystem lift = new LiftSubsystem();
        ClawSubsystem claw = new ClawSubsystem();
        IntakeSubsystem intake = new IntakeSubsystem(telemetry1);
        CustomLocalization l = new CustomLocalization(startPos, hardwareMap);
        DriveSubsystem drive = new DriveSubsystem(l, telemetry1);

        Robot.hasCachedLiftValues = false; // TODO
        Robot.robotInit(hardwareMap, l, telemetry1, intake, claw, lift);
        Robot.onlyLogImportant = true;

        OpenCvCamera cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "outtake_camera"));
        TeamPropDetectionPipeline pipeline = new TeamPropDetectionPipeline(cam, telemetry1, true);

        OpenCvCamera tagCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "tag_camera"));
        BoardTagLocalizationPipeline tagPipeline = new BoardTagLocalizationPipeline(tagCam);

        Robot.write = new WriteThread(this);
        Robot.writeThread = new Thread(Robot.write);

        telemetry1.add("Initialization", "done");

        Robot.telemetry.update();

        waitForStart();

        Robot.writeThread.start();

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

        if (last == TeamPropPosition.right) last = TeamPropPosition.middle;
        else if (last == TeamPropPosition.middle) last = TeamPropPosition.left;
        else last = TeamPropPosition.right;

        pipeline.destroy();
        cam.stopStreaming();

        tagCam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);

        intake.pixelPassCount = 2;
        CommandScheduler.getInstance().schedule(
            new SequentialCommandGroup(
                new WaitCommand(13000),// =================================================================================================
                // go to spike strip and place
// =================================================================================================
                new ParallelCommandGroup(
                    // go to spike strip
                    new DriveToSpikeStripRedTruss(last,
                        new SequentialCommandGroup(
                            new UpdateClaw(Robot.clawSubsystem, ClawSubsystem.ClawState.OPEN),
                            new WaitCommand(400),
                            new InstantCommand(() -> Robot.intakeSubsystem.pixelPassCount = 1))),
                    // prep lift
                    new GoToHeight(lift, Robot.clawSubsystem, 2, ClawSubsystem.ClawState.OPENONE),
                    // make sure team prop doesnt interfere
                    new SequentialCommandGroup(
                        new InstantCommand(() -> Robot.intakeSubsystem.raise()),
                        new WaitCommand(1_000),
                        new InstantCommand(() -> Robot.intakeSubsystem.setPower(1)))),

// =================================================================================================
                // now pick up an extra pixel
// =================================================================================================
                new IntakePixelFromStack(1, 2000, 5, -0.9),

// =================================================================================================
                // move to back board
// =================================================================================================
                new ParallelCommandGroup(
                    new DriveToBackBoardRedTruss(last, 0),
                    // clean up intake
                    new SequentialCommandGroup(
                        new InstantCommand(() -> Robot.intakeSubsystem.setPower(1)),
                        new WaitCommand(500),
                        new InstantCommand(() -> Robot.intakeSubsystem.raise(0))),
                    // prep lift
                    new SequentialCommandGroup(
                        new WaitCommand(2_500),
                        new GoToHeight(lift, claw, 2))),

// =================================================================================================
                // drop pixel (2+1)
// =================================================================================================
                new GoToHeight(lift, claw, 3),
                new InstantCommand(Robot::cacheLiftValues),

                new RamBoard(),
                new UpdateClaw(Robot.clawSubsystem, ClawSubsystem.ClawState.OPENONE),
                new WaitCommand(500),
                new InstantCommand(() -> Robot.clawSubsystem.setWrist(ClawSubsystem.zero.wrist + 0.083333 + 0.02)),
                new WaitCommand(400),
                new UpdateClaw(Robot.clawSubsystem, ClawSubsystem.ClawState.OPEN),
                new WaitCommand(500)

// =================================================================================================
                // cycle to stack
// =================================================================================================
//                new DriveFromBoardToStackRed(last, 1000),

// =================================================================================================
                // intake from stack
// =================================================================================================
//                new IntakePixelFromStack(2, 2000, 3, -0.7),
//                new ParallelCommandGroup(
//                    new DriveToBackBoardRedTruss(last, 1),
//                    // clean up intake
//                    new SequentialCommandGroup(
//                        new InstantCommand(() -> Robot.intakeSubsystem.setPower(1)),
//                        new WaitCommand(500),
//                        new InstantCommand(() -> Robot.intakeSubsystem.raise(0))),
//                    // prep lift
//                    new SequentialCommandGroup(
//                        new WaitCommand(2_500),
//                        new GoToHeight(lift, claw, 2))),

// =================================================================================================
                // return to board for 2+3
// =================================================================================================
//                new GoToHeight(lift, claw, 4),
//                new InstantCommand(Robot::cacheLiftValues),
//                new RamBoard(),
//                new UpdateClaw(Robot.clawSubsystem, ClawSubsystem.ClawState.OPENONE),
//                new WaitCommand(250),
//                new InstantCommand(() -> Robot.clawSubsystem.setWrist(ClawSubsystem.zero.wrist + 0.083333 + 0.02)),
//                new WaitCommand(500),
//                new UpdateClaw(Robot.clawSubsystem, ClawSubsystem.ClawState.OPEN)
            )
        );

        while (opModeIsActive() && !isStopRequested()) {
            Robot.telemetry.addImportant("Detected prop pos from auto", last);

            CommandScheduler.getInstance().run();
            Robot.update();
        }
    }
}
