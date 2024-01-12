package org.firstinspires.ftc.teamcode.FTC.Subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;

@Config
public class LiftSubsystem extends SubsystemBase {
    public static boolean safeRegion = false;
    public static double safetyThreshold = inchesToTicks(3); //TODO: DON'T FORGET SAFETY THRESHOLD
    private static double circumfrence = 4;
    private static double PPR = 384.5; //ticks per revolution of the motor
    public static double P = 0.003, I = 0.145, D = 2, F = 0.0;
    double targetPos = 0;
    public static int index1 = 0;
    public double maxPower = 0.8;
    public static boolean hangOverride = false;


    public PIDFController pid = new PIDFController(P, I, D, F);

    // in ticks
    // 0 -> intake
    // 1 -> slightly above pixel, in tray
    // 2 -> ground
    // 3 -> board row 1
    // 4 -> board row 2
    // 5 -> board row 3...
    public int offset = 0;
    public static double[] rowHeights = new double[]{20, 220, 350, 500, 794, 1357, 1357, 1884, 2365};

    public LiftSubsystem() {
        pid.setTolerance(10);
        index1 = 0;
        safeRegion = false;
    }

    public int updateRow(int index) { // returns new the level
        pid.reset();
        if (index < 0) index = rowHeights.length - 1;
        else if (index >= rowHeights.length) index = 0;

        index1 = index;
        targetPos = rowHeights[index];

        return index;
    }

    public void setTargetPos(double height) {
        pid.reset();
        targetPos = height;
    } //height is in ticks

    @Override
    public void periodic() {
        if (hangOverride) return;

        safeRegion = Math.abs(Robot.liftEncoder.getCurrentPosition()) > safetyThreshold;
        double controllerPower = -pid.calculate(targetPos, read());
        controllerPower = Math.signum(controllerPower) * Math.min(Math.abs(controllerPower), maxPower);
        Robot.telemetry.add("lift pid power", controllerPower);
        Robot.telemetry.add("lift target pos", targetPos);
        Robot.telemetry.add("lift index", index1);

        setPower(Range.clip(controllerPower + F, -1, 1));
    }

    public void setPower(double power) {
        Robot.motor1.setPower(power);
        Robot.motor2.setPower(-power);
    }


    public double read() { return -1 * Robot.liftEncoder.getCurrentPosition()-offset; }

    public static double inchesToTicks(double inches) { return PPR * inches / (circumfrence); }
}
