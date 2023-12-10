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
    public static DcMotor motor1, motor2, intakeMotor, liftEncoder;
    public static Servo clawBottom, clawTop, intakeServo1, intakeServo2, wrist1, wrist2, arm1, arm2,drone;

    public static CRServo intakeRoller;
    public static DistanceSensor distance;
    public static DistanceSensor distance1;
    public static DistanceSensor distance2;//this is left of robot
    public static boolean pastTruss = false;
    public static CustomLocalization l;
    public static HardwareMap hardwareMap;
    public static double distanceBetween = 96.0 * 2;
    public static double t = 0.0;
    public static int autoLiftPos=0;
    public static boolean isBusy = false;
    public static LoggerTool telemetry;
    public static LiftSubsystem lift;
    public static ClawSubsystem claw;
    public static boolean forwardIsForward=true;
    public static IntakeSubsystem intakeSubsystem;
    public static int level = 0; // this is automatically controlled by GoToHeight, so dont touch it!

    public static void robotInit(HardwareMap hardwareMap, CustomLocalization _l, LoggerTool _telemetry, IntakeSubsystem _intakeSubsystem, ClawSubsystem _claw) {
        l = _l;
        intakeRoller = hardwareMap.crservo.get("intakeRoller");
        drone=hardwareMap.servo.get("drone");
        //lift=_lift;
        intakeSubsystem = _intakeSubsystem;
        liftEncoder = hardwareMap.dcMotor.get("rightFront");
        claw = _claw;
        telemetry = _telemetry;
        Robot.hardwareMap = hardwareMap;
        distance1 = hardwareMap.get(DistanceSensor.class, "dist1");
        distance2 = hardwareMap.get(DistanceSensor.class, "dist2");

        intakeMotor = Robot.hardwareMap.dcMotor.get("intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm1 = Robot.hardwareMap.servo.get("arm1");
        arm2 = Robot.hardwareMap.servo.get("arm2");
        wrist1 = Robot.hardwareMap.servo.get("wrist1");
        wrist2 = Robot.hardwareMap.servo.get("wrist2");
        intakeServo1 = hardwareMap.servo.get("intakeServo1");
        intakeServo2 = hardwareMap.servo.get("intakeServo2");
        motor1 = hardwareMap.dcMotor.get("lift1"); // control 2
        motor2 = hardwareMap.dcMotor.get("lift2"); // expansion 3
        clawTop = hardwareMap.get(Servo.class, "clawTop");
        clawBottom = hardwareMap.get(Servo.class, "clawBottom");
        //distance = hardwareMap.get(DistanceSensor.class, "distance");

        // have to manually reset values because ftc dashboard (i think?) causes static vars to stay across runs
        level = 0;
    }

    public static void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    public static boolean isPastTruss() {
        return (pastTruss);
    }
}
