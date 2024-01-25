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
    public static double intakeDownPosition = 0;
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
        //if (Constants.robotPose.getY() > 100) Robot.pastTruss = true;
        //else Robot.pastTruss = false;

        if (Robot.hardware.lastIntakeDist < intakeDistThreshold) intakeDistFrameCount++;
        else intakeDistFrameCount = 0;

        if (intakeDistFrameCount >= intakeDistFrameThreshold) prediction = IntakeDistancePrediction.FILLED;
        else if (prediction == IntakeDistancePrediction.FILLED) {
            pixelPassCount++;
            prediction = IntakeDistancePrediction.EMPTY;
        }

        telemetry.addImportant("intake pixel pass count", pixelPassCount);
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
