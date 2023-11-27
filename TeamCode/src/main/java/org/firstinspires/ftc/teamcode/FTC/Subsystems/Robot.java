package org.firstinspires.ftc.teamcode.FTC.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Robot {
    public static DcMotor motor1, motor2, intakeMotor;
    public static Servo claw1, claw2, intake, wrist1, wrist2, arm1, arm2;
    public static DistanceSensor distance;
    public static DistanceSensor distance1;
    public static DistanceSensor distance2;//this is left of robot
    public static boolean pastTruss = false;
    public static HardwareMap hardwareMap;
    public static double distanceBetween = 96.0 * 2;
    public static double t = 0.0;
    public static boolean isBusy = false;

    public static void robotInit(HardwareMap hardwareMap) {
        Robot.hardwareMap = hardwareMap;
        distance1 = hardwareMap.get(DistanceSensor.class, "dist1");
        distance2 = hardwareMap.get(DistanceSensor.class, "dist2");

        intakeMotor = Robot.hardwareMap.dcMotor.get("intakeMotor");
        //intake = hardwareMap.servo.get("intake");
        //motor1 = hardwareMap.dcMotor.get("liftMotor1");
        //motor2 = hardwareMap.dcMotor.get("liftMotor2");
        //distance = hardwareMap.get(DistanceSensor.class, "distance");

    }

    public static void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    public static boolean isPastTruss() {
        return (pastTruss);
    }
}
