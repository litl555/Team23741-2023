package org.firstinspires.ftc.teamcode.FTC.Subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;
import java.util.List;

public class LiftSubsystem extends SubsystemBase {
    HardwareMap hardwareMap;
    public static boolean safeRegion = false;
    public static double safetyThreshold = inchesToTicks(3);
    private static double circumfrence = 4;
    private static double PPR = 384.5;
    public List<Double> rowHeights = Arrays.asList(0.0, 1.0, 2.0, 3.0, 4.0);
    double P = 0.0;
    double I = 0.0;
    double D = 0.0;
    double F = 0.0;
    double targetPos = 0;
    public static int index1 = 0;
    BasicPID controller = new BasicPID(new PIDCoefficients(P, I, D));

    public LiftSubsystem() {
        safeRegion = false;
    }

    public void updateRow(int index) {
        index1 = index;
        targetPos = rowHeights.get(index);

    }

    public void setTargetPos(double height) {
        targetPos = height;
    }

    @Override
    public void periodic() {
        if (Robot.motor1.getCurrentPosition() > safetyThreshold) {
            safeRegion = true;
        } else {
            safeRegion = false;
        }
//        double controllerPower = controller.calculate(inchesToTicks(targetPos), read());
//        setPower(Range.clip(controllerPower + F, -1, 1));
    }

    public void setPower(double power) {
        Robot.motor1.setPower(power);
        Robot.motor2.setPower(-power);
    }

    public double read() {
        return (Robot.motor2.getCurrentPosition());
    }

    public static double inchesToTicks(double inches) {
        return (PPR * inches / (circumfrence));
    }

}
