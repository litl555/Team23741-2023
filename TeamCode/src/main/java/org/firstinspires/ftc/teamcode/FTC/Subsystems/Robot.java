package org.firstinspires.ftc.teamcode.FTC.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;
import org.firstinspires.ftc.teamcode.FTC.Localization.OdometryModule;
import org.firstinspires.ftc.teamcode.FTC.SmallParticle.ParticleRev2M;
import org.firstinspires.ftc.teamcode.FTC.Threading.HardwareThread;
import org.firstinspires.ftc.teamcode.FTC.Threading.MathThread;

import java.lang.reflect.Field;


@Config
public class Robot {
    // ==============================================================
    // +                           CONFIG                           =
    // ==============================================================
    // note that these are made public just in case, but you should really go through .hardware to read/write
    // (also setting up a whole ass hierarchy would be very painful)

    // drivetrain
    public static DcMotorEx leftFront, leftRear, rightFront, rightRear;
    public static OdometryModule leftPod, rightPod, backPod;

    // lift
    public static DcMotorEx liftLeft, liftRight;
    public static DcMotorEx liftEncoder;

    // intake
    public static CRServo bottomRoller;
    public static DistanceSensor intakeDist;
    public static DcMotorEx intakeMotor;
    public static Servo droptakeRight, droptakeLeft;

    // arm
    public static Servo armYellow, armGreen;

    // wrist
    public static Servo wristRed, wristBlue;

    // claw
    public static Servo clawBlack, clawWhite;

    public static DcMotorEx drone;

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
    public static boolean onlyLogImportant = false;

    // ==============================================================
    // +                         SUBSYSTEMS                         =
    // ==============================================================
    public static CustomLocalization customLocalization;
    public static LoggerTool telemetry;
    public static HardwareMap hardwareMap;
    public static ClawSubsystem clawSubsystem;
    public static LiftSubsystem liftSubsystem;
    public static IntakeSubsystem intakeSubsystem;
    public static HardwareThread hardware;

    public static MathThread math;
    public static Thread hardwareThread, mathThread;

    public static LinearOpMode caller;


    public static void robotInit(HardwareMap hardwareMap, CustomLocalization _l, LoggerTool _telemetry, IntakeSubsystem intake, ClawSubsystem _claw, LiftSubsystem _lift) {
        customLocalization = _l;
        clawSubsystem = _claw;
        liftSubsystem = _lift;
        intakeSubsystem = intake;
        telemetry = _telemetry;
        Robot.hardwareMap = hardwareMap;

        // drivetrain
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftPod = new OdometryModule(hardwareMap.dcMotor.get("intake"));
        rightPod = new OdometryModule(hardwareMap.dcMotor.get("drone"));
        backPod = new OdometryModule(hardwareMap.dcMotor.get("rightRear"));

        rightPod.reverse();

        backPod.reset();
        rightPod.reset();
        leftPod.reset();

        // lift
        liftEncoder = hardwareMap.get(DcMotorEx.class,"liftLeft");
        liftLeft = hardwareMap.get(DcMotorEx.class,"liftLeft");
        liftRight = hardwareMap.get(DcMotorEx.class,"liftRight");

        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // intake
        bottomRoller = hardwareMap.crservo.get("bottomRoller");
        intakeDist = hardwareMap.get(DistanceSensor.class, "intakeDist");
        intakeMotor = hardwareMap.get(DcMotorEx.class,"intake");

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

        // drone
        drone = hardwareMap.get(DcMotorEx.class, "drone");

        // threads
        hardware = new HardwareThread(hardwareMap);
        math = new MathThread();
        hardwareThread = new Thread(hardware, "Hardware Thread");
        mathThread = new Thread(math, "Math Thread");

        // reset static variables where needed
        level = 0;
        forwardIsForward = true;
        onlyLogImportant = false;
        isBusy = false;

        // attach custom distance sensor
        ParticleRev2M dist = null;
        for (HardwareDevice device : hardwareMap.getAll(HardwareDevice.class)) {
            if (device instanceof Rev2mDistanceSensor) {
                try {
                    I2cDeviceSynch tmp = (I2cDeviceSynch) getField(device.getClass(), "deviceClient").get(device);
                    boolean owned = (boolean) getField(device.getClass(), "deviceClientIsOwned").get(device);

                    dist = new ParticleRev2M(tmp, owned);
                    dist.setRangingProfile(ParticleRev2M.RANGING_PROFILE.HIGH_SPEED);

                    String s = (String) hardwareMap.getNamesOf(device).toArray()[0];
                    hardwareMap.remove(s, device);
                    hardwareMap.put(s, dist);
                } catch (Exception e) { Robot.telemetry.addError("Error Attaching Distance Sensor", e); }
            }
        }
    }

    public static void update() {
        // calling it this way makes sure that the thread still has access to the hardwareThread class itself
        // and the instance variables stored within
        if (!hardware.isRunning.get()) hardwareThread.start();

        long t = System.currentTimeMillis();
        hardware.errorHandler.update(t);
        math.errorHandler.update(t);

        Robot.telemetry.update();
    }

    public static boolean isPastTruss() {
        return (pastTruss);
    }

    private static Field getField(Class clazz, String fieldName) {
        try {
            Field f = clazz.getDeclaredField(fieldName);
            f.setAccessible(true);
            return f;
        } catch (NoSuchFieldException e) {
            Class superClass = clazz.getSuperclass();
            if (superClass != null) {
                return getField(superClass, fieldName);
            }
        }
        return null;
    }
}
