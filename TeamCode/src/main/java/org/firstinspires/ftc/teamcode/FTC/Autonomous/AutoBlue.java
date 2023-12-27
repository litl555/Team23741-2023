package org.firstinspires.ftc.teamcode.FTC.Autonomous;

import static org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot.liftEncoder;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.FTC.Commands.DriveToBackBoardBlue;
import org.firstinspires.ftc.teamcode.FTC.Commands.DriveToBackBoardRed;
import org.firstinspires.ftc.teamcode.FTC.Commands.DriveToSpikeStripBlue;
import org.firstinspires.ftc.teamcode.FTC.Commands.DriveToSpikeStripRed;
import org.firstinspires.ftc.teamcode.FTC.Commands.GoToHeight;
import org.firstinspires.ftc.teamcode.FTC.Commands.ParkBlue;
import org.firstinspires.ftc.teamcode.FTC.Commands.ParkRed;
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
        Robot.lift = lift;

        IntakeSubsystem intake = new IntakeSubsystem(telemetry1);

        //SampleMecanumDrive dr = new SampleMecanumDrive(hardwareMap);
        CustomLocalization l = new CustomLocalization(startPos, hardwareMap);
        DriveSubsystem drive = new DriveSubsystem(l, telemetry1);

        Robot.robotInit(hardwareMap, l, telemetry1, intake, claw);
        liftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        OpenCvCamera cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "outtake_camera"));
        TeamPropDetectionPipeline pipeline = new TeamPropDetectionPipeline(cam, telemetry1, false);

        Robot.intakeSubsystem.setIntakePosition(IntakeSubsystem.IntakePosition.DOWN);
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
        int similarityCount = 0;
        TeamPropPosition last = TeamPropPosition.undefined;
        while (similarityCount < 10) {
            if (last != pipeline.propPos || pipeline.propPos == TeamPropPosition.undefined) {
                similarityCount = 0;
            } else {
                similarityCount++;
            }
            last = pipeline.propPos;
        }

        if (last == TeamPropPosition.middle) {
            last = TeamPropPosition.left;
        } else if (last == TeamPropPosition.right) {
            last = TeamPropPosition.middle;
        } else {
            last = TeamPropPosition.right;
        }

        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new DriveToSpikeStripBlue(last),
                                new GoToHeight(lift, Robot.claw, 2)
                        ),
                        new WaitCommand(1000),
                        new UpdateClaw(Robot.claw, ClawSubsystem.ClawState.OPENONE),
                        new WaitCommand(500),
                        new ParallelCommandGroup(
                                new GoToHeight(lift, Robot.claw, 3),
                                new DriveToBackBoardBlue(last)
                        ),
                        new RamBoard(),
                        new UpdateClaw(Robot.claw, ClawSubsystem.ClawState.OPEN),
                        new WaitCommand(500),
                        new ParkBlue()
                )
        );
        pipeline.destroy();

        while (opModeIsActive() && !isStopRequested()) {
//            Robot.telemetry.add("loop",(Constants.toSec(Constants.getTime())-Constants.lastTime1)*1000.0);
//            Constants.lastTime1=Constants.toSec(Constants.getTime());
            Robot.autoLiftPos = (int) lift.read();
            Robot.telemetry.add("Detected prop pos from auto", pipeline.propPos);
            Robot.telemetry.add("pose", Constants.robotPose);
            Robot.l.update();
            Robot.telemetry.update();

            CommandScheduler.getInstance().run();
            //lift.periodic();
        }
    }
}
