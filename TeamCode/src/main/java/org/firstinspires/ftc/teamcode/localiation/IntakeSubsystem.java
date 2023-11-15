package org.firstinspires.ftc.teamcode.localiation;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

public class IntakeSubsystem extends SubsystemBase {
    HardwareMap hardwareMap;
    List<Pixel> pixelsLoaded = new ArrayList<>();
    ColorSensor colorSensorTop, colorSensorBottom;
    DcMotor intakeMotor;
    double intakePower = 1;
    double outtakePower = -1;

    public enum IntakePosition {
        UP,
        DOWN
    }

    public enum IntakePowerSetting {
        INTAKE,
        IDLE,
        OUTTAKE
    }

    Servo intake;
    public double intakeUpPosition = 0.0;
    public double intakeDownPosition = 1.0;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        intake = hardwareMap.servo.get("intake");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        colorSensorBottom = hardwareMap.colorSensor.get("c1");
        colorSensorTop = hardwareMap.colorSensor.get("c2");
    }

    public void getPixelsInIntake() {


    }

    public void update(IntakePowerSetting power) {
        switch (power) {
            case INTAKE:
                intakeMotor.setPower(intakePower);
            case IDLE:
                intakeMotor.setPower(0.0);
            case OUTTAKE:
                intakeMotor.setPower(outtakePower);

        }
    }

    public void setIntakePosition(IntakePosition pos) {
        switch (pos) {
            case UP:
                intake.setPosition(intakeUpPosition);
            case DOWN:
                intake.setPosition(intakeDownPosition);
        }
    }

}
