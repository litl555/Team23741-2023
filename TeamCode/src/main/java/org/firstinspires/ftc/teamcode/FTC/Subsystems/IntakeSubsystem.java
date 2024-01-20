package org.firstinspires.ftc.teamcode.FTC.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

@Config
public class IntakeSubsystem extends SubsystemBase {
    public int pixelPassCount = 0;
    public boolean seePixel = false;
    // TODO: remove, we just use setPower instead and its better to have a gradual increase in power rather then constant
    public static double intakePower = 1;
    public static double outtakePower = -.8;

    public static double intakeUpPosition = 0.7;
    private static double intakeDownPosition = 0;
    private LoggerTool telemetry;

    private IntakeDistancePrediction prediction = IntakeDistancePrediction.EMPTY;
    private final double intakeDistThreshold = 115;
    private final int intakeDistFrameThreshold = 1;
    private int intakeDistFrameCount = 0;

    public IntakeSubsystem(LoggerTool telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        // TODO: remove, not needed/should be in better place
        if (Constants.robotPose.getY() > 100) Robot.pastTruss = true;
        else Robot.pastTruss = false;

        double distance = Robot.intakeDist.getDistance(DistanceUnit.MM);

        if (distance < intakeDistThreshold) intakeDistFrameCount++;
        else intakeDistFrameCount = 0;

        if (intakeDistFrameCount >= intakeDistFrameThreshold) prediction = IntakeDistancePrediction.FILLED;
        else if (prediction == IntakeDistancePrediction.FILLED) {
            pixelPassCount++;
            prediction = IntakeDistancePrediction.EMPTY;
        }

        telemetry.add("intake distance", distance);
        telemetry.add("intake pixel pass count", pixelPassCount);
    }

    public void update(IntakePowerSetting powerset) { // TODO: remove, just call setPower()
        telemetry.add("powerset", powerset);
        switch (powerset) {

            case INTAKE:
                Robot.setIntakePower(intakePower);
                Robot.bottomRoller.setPower(-1.0);
                break;
            case IDLE:
                telemetry.add("powering off", "yes");
                Robot.setIntakePower(0.0);
                Robot.bottomRoller.setPower(0.0);
                telemetry.add("power2", Robot.intakeMotor.getPower());
                break;
            case OUTTAKE:
                telemetry.add("powerset1", powerset);
                telemetry.add("outtaking", "outtaking");
                Robot.setIntakePower(outtakePower);
                Robot.bottomRoller.setPower(1.0);
                break;

        }
    }

    public void setPower(double power) {
        Robot.intakeMotor.setPower(power);
        Robot.bottomRoller.setPower(power);
    }

    public void resetPixel() {
        pixelPassCount = 0;
    }

    public double getDistance() {
        return (Robot.intakeDist.getDistance(DistanceUnit.MM));
    }

    public void setIntakePosition(IntakePosition pos) {
        switch (pos) {
            case UP:
                Robot.droptakeRight.setPosition(1.0 - intakeUpPosition);
                Robot.droptakeLeft.setPosition(intakeUpPosition);
                break;
            case DOWN:
                Robot.droptakeRight.setPosition(1.0 - intakeDownPosition);
                Robot.droptakeLeft.setPosition(intakeDownPosition);
                break;
        }
    }

    public enum IntakePosition {
        UP,
        DOWN
    }

    public enum IntakePowerSetting {
        INTAKE,
        IDLE,
        OUTTAKE
    }

    private enum IntakeDistancePrediction {
        FILLED, EMPTY
    }
}
