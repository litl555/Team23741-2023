package org.firstinspires.ftc.teamcode.FTC.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.Range;

@Config
public class LiftSubsystem extends SubsystemBase {
    public final double P = 0.003, I = 0.145, D = 2, F = 0.0;
    public PIDFController pid = new PIDFController(P, I, D, F);

    public double maxPower = 1;
    public boolean hangOverride = false;
    public double controllerPower = 0;

    // in ticks
    // 0 -> intake
    // 1 -> slightly above pixel, in tray
    // 2 -> ground
    // 3 -> board row 1
    // 4 -> board row 2
    // 5 -> board row 3...
    public double[] rowHeights = new double[]{-30, 280, 425, 650, 800, 1200, 1600, 2000, 2300};

    public LiftSubsystem() {
        pid.setTolerance(10);
    }

    public int updateRow(int index) { // returns new the level
        pid.reset();
        if (index < 0) index = rowHeights.length - 1;
        else if (index >= rowHeights.length) index = 0;

        targetPos = rowHeights[index];

        return index;
    }

    public double targetPos = 0; // in ticks
    public void setTargetPos(double height) {
        pid.reset();
        targetPos = height;
    }

    @Override
    public void periodic() {
        if (hangOverride) return;

        Robot.telemetry.add("lift pid power", controllerPower);
        Robot.telemetry.add("lift target pos", targetPos);

        setPower(Range.clip(controllerPower + F, -1, 1));
    }

    public void calculateControllerPower() {
        if (hangOverride) return;

        controllerPower = pid.calculate(targetPos, Robot.hardware.lastLiftPosition);
        controllerPower = Math.signum(controllerPower) * Math.min(Math.abs(controllerPower), maxPower);
    }

    public void setPower(double power) { Robot.hardware.setLiftPower(power); }
}
