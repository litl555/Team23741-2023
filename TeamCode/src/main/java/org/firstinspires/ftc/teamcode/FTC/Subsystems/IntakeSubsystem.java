package org.firstinspires.ftc.teamcode.FTC.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerData;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;

import java.util.concurrent.atomic.AtomicBoolean;

@Config
public class IntakeSubsystem extends SubsystemBase {
    public int pixelPassCount = 0;
    public boolean seePixel = false;

    // down, stack 1, stack 2, stack 3, stack 4, stack 5, up
    // 0.2 -> stack 3
    // 0.18 -> stack 2
    // 0 -> stack 1
    public static double[] droptakeLevel = new double[] {0, 0, 0.04, 0.075, 0.088, 0.1, 0.5};

    private LoggerTool telemetry;

    private IntakeDistancePrediction prediction = IntakeDistancePrediction.EMPTY;
    private final double intakeDistThreshold = 113;
    private final int intakeDistFrameThreshold = 1;
    private int intakeDistFrameCount = 0;
    public double intakeIndex = 0;
    public AtomicBoolean activateIntakeDist = new AtomicBoolean(false);
    public Gamepad rumblePad = null;

    public IntakeSubsystem(LoggerTool telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        if (!activateIntakeDist.get() || Robot.hardware.lastIntakeDist == -1) {
            telemetry.addImportant(new LoggerData("Is Active", "false", "INTAKE"));
            return;
        } else {
            telemetry.addImportant(new LoggerData("Is Active", "true", "INTAKE"));
            telemetry.addImportant(new LoggerData("Detected Distance", Robot.hardware.lastIntakeDist, "INTAKE"));
        }

        if (Robot.hardware.lastIntakeDist < intakeDistThreshold) intakeDistFrameCount++;
        else intakeDistFrameCount = 0;

        if (intakeDistFrameCount >= intakeDistFrameThreshold) {
            //if (prediction == IntakeDistancePrediction.EMPTY) pixelPassCount++;
            prediction = IntakeDistancePrediction.FILLED;
        } else if (prediction == IntakeDistancePrediction.FILLED) {
            pixelPassCount++;
            prediction = IntakeDistancePrediction.EMPTY;

            if (rumblePad != null) rumblePad.rumble(200);
        }

        telemetry.addImportant(new LoggerData("Pixel Count", pixelPassCount, "INTAKE"));
    }

    public void raise() {
        this.setDroptakePosition(droptakeLevel[droptakeLevel.length - 1]);
    }

    public void raise(double p) {
        raise();
        this.setPower(p);
    }

    public void setPower(double power) { Robot.hardware.setIntakePower(power); }

    public void resetPixel() {
        pixelPassCount = 0;
    }

    // TODO: remove
    public double getDistance() {
        return (Robot.intakeDist.getDistance(DistanceUnit.MM));
    }

    public void setDroptakePosition(double angle) {
        Robot.hardware.setDroptake(angle);
        intakeIndex = angle;
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
