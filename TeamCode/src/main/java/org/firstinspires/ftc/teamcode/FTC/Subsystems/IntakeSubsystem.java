package org.firstinspires.ftc.teamcode.FTC.Subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;

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
    LoggerTool telemetry;

    public IntakeSubsystem(LoggerTool telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        if (Constants.robotPose.getY() > 100) {
            Robot.pastTruss = true;
        } else {
            Robot.pastTruss = false;
        }
        telemetry.add("powerCurrent", Robot.intakeMotor.getPower());
    }

    public void getPixelsInIntake() {


    }

    public void update(IntakePowerSetting powerset) {
        telemetry.add("powerset", powerset);
        switch (powerset) {

            case INTAKE:
                Robot.setIntakePower(intakePower);
                break;
            case IDLE:
                telemetry.add("powering off", "yes");
                Robot.setIntakePower(0.0);
                telemetry.add("power2", Robot.intakeMotor.getPower());
                break;
            case OUTTAKE:
                telemetry.add("powerset1", powerset);
                telemetry.add("outtaking", "outtaking");
                Robot.setIntakePower(outtakePower);
                break;

        }
    }

    public void setPower(double power) {
        Robot.intakeMotor.setPower(power);

    }

    public void resetPixel() {
        pixelPassCount = 0;
    }

    public double getDistance() {
        return (Robot.distance.getDistance(DistanceUnit.MM));
    }

    public void setIntakePosition(IntakePosition pos) {
        switch (pos) {
            case UP:
                Robot.intake.setPosition(intakeUpPosition);
            case DOWN:
                Robot.intake.setPosition(intakeDownPosition);
        }
    }

}
