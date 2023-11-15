package org.firstinspires.ftc.teamcode.FTC.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

public class IntakeSubsystem extends SubsystemBase {
    HardwareMap hardwareMap;
    List<Pixel> pixelsLoaded = new ArrayList<>();
    ColorSensor colorSensorTop, colorSensorBottom;
    public int pixelPassCount = 0;
    public boolean seePixel = false;
    DcMotor intakeMotor;
    double intakePower = 1;
    double outtakePower = -1;
    Robot robot;

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

    public IntakeSubsystem(Robot robot) {
        this.robot = robot;
    }

    public void getPixelsInIntake() {


    }

    public void update(IntakePowerSetting power) {
        switch (power) {
            case INTAKE:
                setPower(intakePower);
            case IDLE:
                setPower(0.0);
            case OUTTAKE:
                setPower(outtakePower);

        }
    }

    public void setPower(double power) {
        robot.intakeMotor.setPower(power);
    }

    public void resetPixel() {
        pixelPassCount = 0;
    }

    public double getDistance() {
        return (robot.distance.getDistance(DistanceUnit.MM));
    }

    public void setIntakePosition(IntakePosition pos) {
        switch (pos) {
            case UP:
                robot.intake.setPosition(intakeUpPosition);
            case DOWN:
                robot.intake.setPosition(intakeDownPosition);
        }
    }

}
