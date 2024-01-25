package org.firstinspires.ftc.teamcode.FTC.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerData;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;
import org.slf4j.Logger;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

@Config
public class IntakeSubsystem extends SubsystemBase {
    public int pixelPassCount = 0;
    public boolean seePixel = false;

    public static double intakeUpPosition = 0.7;
    public static double intakeDownPosition = 0;
    private LoggerTool telemetry;

    private IntakeDistancePrediction prediction = IntakeDistancePrediction.EMPTY;
    private final double intakeDistThreshold = 115;
    private final int intakeDistFrameThreshold = 1;
    private int intakeDistFrameCount = 0;
    public AtomicBoolean activateIntakeDist = new AtomicBoolean(false);

    public IntakeSubsystem(LoggerTool telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        if (!activateIntakeDist.get() || Robot.hardware.lastIntakeDist == -1) {
            telemetry.addImportant(new LoggerData("Is Active", "false", "INTAKE"));
            return;
        } else telemetry.addImportant(new LoggerData("Is Active", "true", "INTAKE"));

        if (Robot.hardware.lastIntakeDist < intakeDistThreshold) intakeDistFrameCount++;
        else intakeDistFrameCount = 0;

        if (intakeDistFrameCount >= intakeDistFrameThreshold) prediction = IntakeDistancePrediction.FILLED;
        else if (prediction == IntakeDistancePrediction.FILLED) {
            pixelPassCount++;
            prediction = IntakeDistancePrediction.EMPTY;
        }

        telemetry.addImportant(new LoggerData("Pixel Count", pixelPassCount, "INTAKE"));
    }

    public void update(IntakePowerSetting powerset) { // TODO: remove, just call setPower()
        // does nothing right not, just temp before we fully remove this func
    }

    public void setPower(double power) { Robot.hardware.setIntakePower(power); }

    public void resetPixel() {
        pixelPassCount = 0;
    }

    // TODO: remove
    public double getDistance() {
        return (Robot.intakeDist.getDistance(DistanceUnit.MM));
    }

    public void setIntakePosition(IntakePosition pos) { Robot.hardware.setDroptake(pos); }

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
