package org.firstinspires.ftc.teamcode.FTC.Autonomous;

import static org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot.liftEncoder;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.FTC.Commands.AutoBlue.Board.DriveToBackBoardBlue;
import org.firstinspires.ftc.teamcode.FTC.Commands.AutoBlue.DriveToParkingBlue;
import org.firstinspires.ftc.teamcode.FTC.Commands.AutoBlue.Board.DriveToSpikeStripBlue;
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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous
public class AutoBlue extends LinearOpMode {
    TeamPropPosition pos;
    public static Pose2d startPos = new Pose2d(300, 1500, Math.PI / 2.0);

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
        Robot.liftSubsystem = lift;

        IntakeSubsystem intake = new IntakeSubsystem(telemetry1);

        CustomLocalization l = new CustomLocalization(startPos, hardwareMap);
        DriveSubsystem drive = new DriveSubsystem(l, telemetry1);

        Robot.robotInit(hardwareMap, l, telemetry1, intake, claw, lift);
        liftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        OpenCvCamera cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "outtake_camera"));
        TeamPropDetectionPipeline pipeline = new TeamPropDetectionPipeline(cam, telemetry1, false);

        //Robot.intakeSubsystem.setIntakePosition(IntakeSubsystem.IntakePosition.DOWN);
        waitForStart();

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

        CommandScheduler.getInstance().schedule(
            new SequentialCommandGroup(
                // go to spike strip
                new ParallelCommandGroup(
                    new DriveToSpikeStripBlue(last),
                    new GoToHeight(lift, Robot.clawSubsystem, 2)),
                // drop pixel
                new UpdateClaw(Robot.clawSubsystem, ClawSubsystem.ClawState.OPENONE),
                new WaitCommand(250),
                // go to board
                new ParallelCommandGroup(
                    new GoToHeight(lift, Robot.clawSubsystem, 3),
                    new DriveToBackBoardBlue(last)),
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
                    new DriveToParkingBlue()))
        );

        while (opModeIsActive() && !isStopRequested()) {
            Robot.telemetry.add("Detected prop pos from auto", pipeline.propPos);
            Robot.telemetry.add("pose", Constants.robotPose);
            Robot.customLocalization.update();
            Robot.telemetry.update();

            CommandScheduler.getInstance().run();
        }
    }
}
