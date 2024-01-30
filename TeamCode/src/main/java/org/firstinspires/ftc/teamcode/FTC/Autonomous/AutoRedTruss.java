package org.firstinspires.ftc.teamcode.FTC.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.FTC.Commands.AutoRed.Truss.DriveToBackBoardRedTruss;
import org.firstinspires.ftc.teamcode.FTC.Commands.AutoRed.DriveToParkingRed;
import org.firstinspires.ftc.teamcode.FTC.Commands.AutoRed.Truss.DriveToSpikeStripRedTruss;
import org.firstinspires.ftc.teamcode.FTC.Commands.AutoRed.Truss.DriveToStackCycle;
import org.firstinspires.ftc.teamcode.FTC.Commands.AutoRed.Truss.DriveToTrussCycle;
import org.firstinspires.ftc.teamcode.FTC.Commands.GoToHeight;
import org.firstinspires.ftc.teamcode.FTC.Commands.IntakePixelFromStack;
import org.firstinspires.ftc.teamcode.FTC.Commands.RamBoard;
import org.firstinspires.ftc.teamcode.FTC.Commands.RamIntake;
import org.firstinspires.ftc.teamcode.FTC.Commands.UpdateClaw;
import org.firstinspires.ftc.teamcode.FTC.Commands.UpdateIntake;
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

@Photon(singleThreadOptimized = false, maximumParallelCommands = 8)
// TODO having so high seems to have caused a few crashes
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

        Robot.robotInit(hardwareMap, l, telemetry1, intake, claw, lift);
        Robot.onlyLogImportant = true;

        OpenCvCamera cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "outtake_camera"));
        TeamPropDetectionPipeline pipeline = new TeamPropDetectionPipeline(cam, telemetry1, true);

        telemetry1.add("Initialization", "done");

        Robot.write = new WriteThread(this);
        Robot.writeThread = new Thread(Robot.write);

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

        intake.pixelPassCount = 2;
        // TODO: add back in two distance sensors, then during board side auto after we place the pixel on board and
        // and prepare to cycle, use the distance sensors to check if there is still a robot on the wall side of the truss

        CommandScheduler.getInstance().schedule(
            new SequentialCommandGroup(
                // go to spike strip and place
            new ParallelCommandGroup(
                new InstantCommand(() -> {
                    Robot.intakeSubsystem.setDroptakePosition(IntakeSubsystem.droptakeLevel[IntakeSubsystem.droptakeLevel.length - 1]);
                }),
                new DriveToSpikeStripRedTruss(last,
                    new SequentialCommandGroup(
                        new UpdateClaw(Robot.clawSubsystem, ClawSubsystem.ClawState.OPEN),
                        new WaitCommand(200),
                        new InstantCommand(() -> Robot.intakeSubsystem.pixelPassCount = 1))),
                new GoToHeight(lift, Robot.clawSubsystem, 2, ClawSubsystem.ClawState.OPENONE),
                new SequentialCommandGroup(
                    new WaitCommand(1_000),
                    new InstantCommand(() -> Robot.intakeSubsystem.setPower(0.7)))),
                // now pick up an extra pixel
                new ParallelCommandGroup(
                    new IntakePixelFromStack(1, 2000, 5),
                    new RamIntake()
                ),
                // move to back board
                new ParallelCommandGroup(
                    // clean up intake
                    new SequentialCommandGroup(
                        new InstantCommand(() -> Robot.intakeSubsystem.setPower(1)),
                        new WaitCommand(500),
                        new InstantCommand(() -> {
                            Robot.intakeSubsystem.setPower(0);
                            Robot.intakeSubsystem.setDroptakePosition(IntakeSubsystem.droptakeLevel[6]);
                        })
                    ),
                    new DriveToBackBoardRedTruss(last, 0),
                    new SequentialCommandGroup(
                        new WaitCommand(2_500),
                        new GoToHeight(lift, claw, 2)
                    )
                ),
                new GoToHeight(lift, claw, 3),
                new RamBoard(),

                new UpdateClaw(Robot.clawSubsystem, ClawSubsystem.ClawState.OPENONE),
                new WaitCommand(250),
                new InstantCommand(() -> Robot.clawSubsystem.setWrist(ClawSubsystem.zero.wrist + 0.083333 + 0.02)),
                new WaitCommand(500),
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
                    new DriveToTrussCycle(last).interruptOn(() -> Robot.liftSubsystem.targetPos == Robot.liftSubsystem.rowHeights[0])),
                new DriveToStackCycle(last),
                new ParallelCommandGroup(
                    new IntakePixelFromStack(2, 2000, 3),
                    new RamIntake()
                ),
                new ParallelCommandGroup(
                    // clean up intake
                    new SequentialCommandGroup(
                        new InstantCommand(() -> Robot.intakeSubsystem.setPower(1)),
                        new WaitCommand(500),
                        new InstantCommand(() -> {
                            Robot.intakeSubsystem.setPower(0);
                            Robot.intakeSubsystem.setDroptakePosition(IntakeSubsystem.droptakeLevel[6]);
                        })
                    ),
                    new DriveToBackBoardRedTruss(last, 1),
                    new SequentialCommandGroup(
                        new WaitCommand(2_500),
                        new GoToHeight(lift, claw, 2)
                    )
                ),
                new GoToHeight(lift, claw, 4),
                new RamBoard(),

                new UpdateClaw(Robot.clawSubsystem, ClawSubsystem.ClawState.OPENONE),
                new WaitCommand(250),
                new InstantCommand(() -> Robot.clawSubsystem.setWrist(ClawSubsystem.zero.wrist + 0.083333 + 0.02)),
                new WaitCommand(500),
                new UpdateClaw(Robot.clawSubsystem, ClawSubsystem.ClawState.OPEN)
            )
        );
        while (opModeIsActive() && !isStopRequested()) {
            Robot.telemetry.addImportant("Detected prop pos from auto", last);

            CommandScheduler.getInstance().run();
            Robot.update();
        }
    }
}
