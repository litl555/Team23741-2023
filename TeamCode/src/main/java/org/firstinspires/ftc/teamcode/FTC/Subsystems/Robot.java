package org.firstinspires.ftc.teamcode.FTC.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    public DcMotor motor1, motor2, intakeMotor;
    public Servo claw1, claw2, intake;
    public DistanceSensor distance;

    public Robot(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        intake = hardwareMap.servo.get("intake");
        motor1 = hardwareMap.dcMotor.get("liftMotor1");
        motor2 = hardwareMap.dcMotor.get("liftMotor2");
        distance = hardwareMap.get(DistanceSensor.class, "distance");

    }
}
