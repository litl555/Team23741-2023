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
    public static DcMotor motor1, motor2, intakeMotor;
    public static Servo clawBottom, clawTop, intakeServo1, intakeServo2, wrist1, wrist2, arm1, arm2;
    public static CRServo intakeRoller;
    public static DistanceSensor distance;
    public static DistanceSensor distance1;
    public static DistanceSensor distance2;//this is left of robot
    public static boolean pastTruss = false;
    public static CustomLocalization l;
    public static HardwareMap hardwareMap;
    public static double distanceBetween = 96.0 * 2;
    public static double t = 0.0;
    public static boolean isBusy = false;
    public static LoggerTool telemetry;
    public static LiftSubsystem lift;
    public static ClawSubsystem claw;
    public static IntakeSubsystem intakeSubsystem;

    public static void robotInit(HardwareMap hardwareMap, CustomLocalization _l, LoggerTool _telemetry, IntakeSubsystem _intakeSubsystem, ClawSubsystem _claw) {
        l = _l;
        intakeRoller = hardwareMap.crservo.get("intakeRoller");
        //lift=_lift;
        _intakeSubsystem = intakeSubsystem;
        claw = _claw;
        telemetry = _telemetry;
        Robot.hardwareMap = hardwareMap;
        distance1 = hardwareMap.get(DistanceSensor.class, "dist1");
        distance2 = hardwareMap.get(DistanceSensor.class, "dist2");

        intakeMotor = Robot.hardwareMap.dcMotor.get("intake");
        intakeServo1 = hardwareMap.servo.get("intakeServo1");
        intakeServo2 = hardwareMap.servo.get("intakeServo2");
        motor1 = hardwareMap.dcMotor.get("lift1");
        motor2 = hardwareMap.dcMotor.get("lift2");
        clawTop = hardwareMap.get(Servo.class, "clawTop");
        clawBottom = hardwareMap.get(Servo.class, "clawBottom");
        //distance = hardwareMap.get(DistanceSensor.class, "distance");

    }

    public static void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    public static boolean isPastTruss() {
        return (pastTruss);
    }
}
