package org.firstinspires.ftc.teamcode.FTC.Subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
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

@Config
public class IntakeSubsystem extends SubsystemBase {
    HardwareMap hardwareMap;
    List<Pixel> pixelsLoaded = new ArrayList<>();
    ColorSensor colorSensorTop, colorSensorBottom;
    public int pixelPassCount = 0;
    public boolean seePixel = false;
    DcMotor intakeMotor;
    public static double intakePower = 1;
    public static double outtakePower = -.8;


    public enum IntakePosition {
        UP,
        DOWN
    }

    public enum IntakePowerSetting {
        INTAKE,
        IDLE,
        OUTTAKE
    }

    public static double intakeUpPosition = 0.0;
    private static double intakeDownPosition = 1.0;
    private LoggerTool telemetry;

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
//        telemetry.add("powerCurrent", Robot.intakeMotor.getPower());
    }

    public void getPixelsInIntake() {


    }

    public void update(IntakePowerSetting powerset) {
        telemetry.add("powerset", powerset);
        switch (powerset) {

            case INTAKE:
                Robot.setIntakePower(intakePower);
                Robot.intakeRoller.setPower(-1.0);
                break;
            case IDLE:
                telemetry.add("powering off", "yes");
                Robot.setIntakePower(0.0);
                Robot.intakeRoller.setPower(0.0);
                telemetry.add("power2", Robot.intakeMotor.getPower());
                break;
            case OUTTAKE:
                telemetry.add("powerset1", powerset);
                telemetry.add("outtaking", "outtaking");
                Robot.setIntakePower(outtakePower);
                Robot.intakeRoller.setPower(1.0);
                break;

        }
    }

    public void setPower(double power) {
        Robot.intakeMotor.setPower(power);
        Robot.intakeRoller.setPower(power);

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
                Robot.intakeServo1.setPosition(1.0 - intakeUpPosition);
                Robot.intakeServo2.setPosition(intakeUpPosition);
                break;
            case DOWN:
                Robot.intakeServo1.setPosition(1.0 - intakeDownPosition);
                Robot.intakeServo2.setPosition(intakeDownPosition);
                break;
        }
    }

}
