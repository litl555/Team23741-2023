package org.firstinspires.ftc.teamcode.FTC.TeleOp;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.FTC.Autonomous.BoardTagLocalizationPipeline;
import org.firstinspires.ftc.teamcode.FTC.Autonomous.TeamPropDetectionPipeline;
import org.firstinspires.ftc.teamcode.FTC.Commands.Drive;
import org.firstinspires.ftc.teamcode.FTC.Commands.GoToHeight;
import org.firstinspires.ftc.teamcode.FTC.Commands.ReturnClawToTray;
import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerData;
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

import java.util.logging.Logger;

@Photon(singleThreadOptimized = false, maximumParallelCommands = 12)
@TeleOp
public class DangerousTeleop extends CommandOpMode {
    private int liftLevel = 0;

    private LiftSubsystem lift;
    private ClawSubsystem claw;
    private IntakeSubsystem intake;
    private DriveSubsystem drive;

    private int droptakeLevel = 0;

    private boolean liftPowerLock = false;

    @Override
    public void initialize() {
        // initialization
        LoggerTool telemetry1 = new LoggerTool(telemetry);
        CommandScheduler.getInstance().reset();

        CustomLocalization l = new CustomLocalization(new Pose2d(0, 0, 0), hardwareMap);

        lift = new LiftSubsystem();                   register(lift);
        claw = new ClawSubsystem();                   register(claw);
        intake = new IntakeSubsystem(telemetry1);   register(intake);
        drive = new DriveSubsystem(l, telemetry1);   register(drive);

        GamepadEx pad1 = new GamepadEx(gamepad1);
        GamepadEx pad2 = new GamepadEx(gamepad2);

        // GAMEPAD 1 CONTROLS
        {
            drive.setDefaultCommand(new Drive(drive, gamepad1));

            pad1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(() -> Robot.forwardIsForward = true));
            pad1.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> Robot.forwardIsForward = false));
            pad1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(() -> intake.setDroptakePosition(IntakeSubsystem.droptakeLevel[6])));
            pad1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(() -> intake.setDroptakePosition(IntakeSubsystem.droptakeLevel[0])));

            // decrement
            pad1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> {
                droptakeLevel = Math.max(0, droptakeLevel - 1);
                Robot.intakeSubsystem.setDroptakePosition(IntakeSubsystem.droptakeLevel[droptakeLevel]);
            }));

            pad1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> {
                droptakeLevel = Math.min(IntakeSubsystem.droptakeLevel.length - 1, droptakeLevel + 1);
                Robot.intakeSubsystem.setDroptakePosition(IntakeSubsystem.droptakeLevel[droptakeLevel]);
            }));

            schedule(new RunCommand(() -> {
                double rt = pad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
                double lt = pad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

                if (rt != 0.0 || lt != 0.0) {
                    intake.rumblePad = gamepad1;
                    intake.activateIntakeDist.set(true);
                    intake.setPower(lt - rt);
                } else {
                    if (Robot.hardware.intakePower != 0) intake.setPower(0);
                    intake.activateIntakeDist.set(false);
                    intake.rumblePad = null;
                }
            }));

            pad1.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(() -> {
                Robot.pidControl = !Robot.pidControl;
                Drive.angledes = Constants.angle;
            }));

            pad1.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(new InstantCommand(() -> {
                if (Robot.intakeSubsystem.intakeIndex == IntakeSubsystem.droptakeLevel[IntakeSubsystem.droptakeLevel.length - 1]) {
                    Robot.intakeSubsystem.setDroptakePosition(IntakeSubsystem.droptakeLevel[0]);
                } else {
                    Robot.intakeSubsystem.setDroptakePosition(IntakeSubsystem.droptakeLevel[IntakeSubsystem.droptakeLevel.length - 1]);

                }
            }));
        }

        // GAMEMPAD 2 CONTROLS
        {
            pad2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new SequentialCommandGroup(
                    new InstantCommand(() -> Robot.drone.setPower(0.5)),
                    new WaitCommand(250),
                    new InstantCommand(() -> Robot.drone.setPower(0))));

            pad2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> {
                    liftLevel++;
                    if (liftLevel > lift.rowHeights.length - 1) liftLevel = 0;
                })
            );

            pad2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> {
                    liftLevel--;
                    if (liftLevel < 0) liftLevel = lift.rowHeights.length - 1;
                })
            );

            pad2.getGamepadButton(GamepadKeys.Button.Y).whenPressed( // go to current lift level
                new InstantCommand(() -> {
                    if (liftLevel == Robot.level) return;

                    // if entering tray make sure to prepare to pick up pixel
                    if (liftLevel == 0 && Robot.level != 1) {
                        schedule(new SequentialCommandGroup(
                            new GoToHeight(lift, claw, 1),
                            new WaitCommand(300),
                            new GoToHeight(lift, claw, 0)
                        ));
                    } else schedule(new GoToHeight(lift, claw, liftLevel));

                    // currently we have a preset to go straight to ground level when picking up from tray, so update liftLevel to reflect that change
                    if (Robot.level == 0 && liftLevel == 1) liftLevel = 2;
                })
            );

            pad2.getGamepadButton(GamepadKeys.Button.X).whenPressed( // go to tray
                new InstantCommand(() -> {
                    liftLevel = 0;
                    if (Robot.level == 0) return;
                    else if (Robot.level == 1) schedule(new GoToHeight(lift, claw, 0));
                    else schedule(new ReturnClawToTray(400));
                })
            );

            pad2.getGamepadButton(GamepadKeys.Button.B).whenPressed( // toggle open
                new InstantCommand(() -> {
                    if (claw.currentState == ClawSubsystem.ClawState.OPENONE)
                        claw.update(ClawSubsystem.ClawState.OPEN);
                    else claw.update(ClawSubsystem.ClawState.OPENONE);
                })
            );

            pad2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> {
                    if (Robot.liftSubsystem.hangOverride) liftPowerLock = true;
                })
            );

            // close claw
            pad2.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> claw.update(ClawSubsystem.ClawState.CLOSED)));

            // toggling how strong the lift should be during hang
            pad2.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(new InstantCommand(() -> {
                lift.hangOverride = true;
                claw.enableHang();
            }));
            pad2.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(new InstantCommand(() -> lift.hangOverride = false));

            schedule(new RunCommand(() -> {
                if (!lift.hangOverride) return;

                if (liftPowerLock) {
                    Robot.telemetry.addImportant("LIFT POWER LOCK", "ACTIVE");
                    lift.setPower(1);
                } else {
                    double y = pad2.getLeftY();
                    if (Math.abs(y) > 0.2) lift.setPower(-y);
                    else lift.setPower(0);
                }
            }));
        }

        Robot.robotInit(hardwareMap, l, telemetry1, intake, claw, lift);
        Robot.onlyLogImportant = true;
        //intake.activateIntakeDist.set(true);

        Robot.write = new WriteThread(this);
        Robot.writeThread = new Thread(Robot.write);

        OpenCvCamera tagCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "tag_camera"));
        BoardTagLocalizationPipeline tagPipeline = new BoardTagLocalizationPipeline(tagCam);

        tagCam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);

        BoardTagLocalizationPipeline.shouldGetPosition = true;

        Robot.telemetry.update();
        waitForStart();

        Robot.writeThread.start();
    }

    @Override
    public void run() {
        Robot.telemetry.addImportant(new LoggerData("LEVEL", liftLevel, "LIFT", true));
        Robot.telemetry.addImportant(new LoggerData("IS OVERRIDDEN", lift.hangOverride, "LIFT", true));
        Robot.telemetry.addImportant(new LoggerData("TICK", Robot.hardware.lastLiftPosition, "LIFT", true));
        Robot.telemetry.addImportant(new LoggerData("POWER", Robot.liftSubsystem.maxPower, "LIFT", true));

        Robot.telemetry.addImportant(new LoggerData("Main", System.currentTimeMillis(), "THREAD UPDATE"));
        Robot.telemetry.addImportant("Droptake", droptakeLevel);


        CommandScheduler.getInstance().run();
        Robot.update();
    }
}
