package org.firstinspires.ftc.teamcode.FTC.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;

@Config
public class Robot {
    // ==============================================================
    // +                           CONFIG                           =
    // ==============================================================
    // main 4 movement motors are controlled by CustomLocalization

    // lift
    public static DcMotor liftLeft, liftRight;
    public static DcMotor liftEncoder;

    // intake
    public static CRServo bottomRoller;
    public static DistanceSensor intakeDist;
    public static DcMotor intakeMotor;
    public static Servo droptakeRight, droptakeLeft;

    // arm
    public static Servo armYellow, armGreen;

    // wrist
    public static Servo wristRed, wristBlue;

    // claw
    public static Servo clawBlack, clawWhite;

    public static DcMotor drone;

    // ==============================================================
    // +                       SHARED VALUES                        =
    // ==============================================================
    public static int level = 0; // this is automatically controlled by GoToHeight, so dont touch it!
    public static boolean forwardIsForward = true;
    public static boolean pastTruss = false;
    public static double distanceBetween = 96.0 * 2;
    public static double t = 0.0;
    public static boolean isBusy = false;
    public static final double width = 412.7; // mm, drivetrain plate to other drivetrain plate
    public static final double length = 440.63; // mm, intake roller to back odo pod u channel

    // ==============================================================
    // +                         SUBSYSTEMS                         =
    // ==============================================================
    public static CustomLocalization customLocalization;
    public static LoggerTool telemetry;
    public static HardwareMap hardwareMap;
    public static ClawSubsystem clawSubsystem;
    public static LiftSubsystem liftSubsystem; // TODO
    public static IntakeSubsystem intakeSubsystem;


    public static void robotInit(HardwareMap hardwareMap, CustomLocalization _l, LoggerTool _telemetry, IntakeSubsystem intake, ClawSubsystem _claw, LiftSubsystem _lift) {
        isBusy = false;

        customLocalization = _l;
        clawSubsystem = _claw;
        liftSubsystem = _lift;
        intakeSubsystem = intake;
        telemetry = _telemetry;
        Robot.hardwareMap = hardwareMap;

        // lift
        liftEncoder = hardwareMap.dcMotor.get("liftLeft");
        liftLeft = hardwareMap.dcMotor.get("liftLeft");
        liftRight = hardwareMap.dcMotor.get("liftRight");

        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // intake
        bottomRoller = hardwareMap.crservo.get("bottomRoller");
        intakeDist = hardwareMap.get(DistanceSensor.class, "intakeDist");
        intakeMotor = Robot.hardwareMap.dcMotor.get("intake");

        droptakeRight = hardwareMap.servo.get("droptakeRight");
        droptakeLeft = hardwareMap.servo.get("droptakeLeft");

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // arm
        armYellow = Robot.hardwareMap.servo.get("armYellow");
        armGreen = Robot.hardwareMap.servo.get("armGreen");

        // wrist
        wristRed = Robot.hardwareMap.servo.get("wristRed");
        wristBlue = Robot.hardwareMap.servo.get("wristBlue");

        // claw
        clawWhite = hardwareMap.servo.get("clawWhite");
        clawBlack = hardwareMap.servo.get("clawBlack");

        drone = hardwareMap.dcMotor.get("drone");

        // reset static variables where needed
        level = 0;
    }

    public static void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    public static boolean isPastTruss() {
        return (pastTruss);
    }
}
