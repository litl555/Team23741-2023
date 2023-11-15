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

    Robot robot;
    public List<Double> rowHeights = Arrays.asList(0.0, 1.0, 2.0, 3.0, 4.0);
    double P = 0.0;
    double I = 0.0;
    double D = 0.0;
    double F = 0.0;
    double targetPos = 0;
    BasicPID controller = new BasicPID(new PIDCoefficients(P, I, D));

    public LiftSubsystem(Robot robot) {
        this.robot = robot;
    }

    public void updateRow(int index) {
        targetPos = rowHeights.get(index);

    }

    public void loop() {
        double controllerPower = controller.calculate(targetPos, read());
        setPower(Range.clip(controllerPower + F, -1, 1));

    }

    public void setPower(double power) {
        robot.motor1.setPower(power);
        robot.motor2.setPower(-power);
    }

    public double read() {
        return (robot.motor2.getCurrentPosition());
    }

}