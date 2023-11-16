package org.firstinspires.ftc.teamcode.FTC.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    public static DcMotor motor1, motor2, intakeMotor;
    public static Servo claw1, claw2, intake, wrist1, wrist2, arm1, arm2;
    public static DistanceSensor distance;
    public static boolean pastTruss = false;

    public Robot(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        intake = hardwareMap.servo.get("intake");
        motor1 = hardwareMap.dcMotor.get("liftMotor1");
        motor2 = hardwareMap.dcMotor.get("liftMotor2");
        distance = hardwareMap.get(DistanceSensor.class, "distance");

    }
}
