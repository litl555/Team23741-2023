package org.firstinspires.ftc.teamcode.localiation;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class LiftSubsystem extends SubsystemBase {
    HardwareMap hardwareMap;
    DcMotor motor1, motor2;
    public List<Double> rowHeights = Arrays.asList(0.0, 1.0, 2.0, 3.0, 4.0);
    double P = 0.0;
    double I = 0.0;
    double D = 0.0;
    double F = 0.0;
    double targetPos = 0;
    BasicPID controller = new BasicPID(new PIDCoefficients(P, I, D));

    public LiftSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        motor1 = hardwareMap.dcMotor.get("liftMotor1");
        motor2 = hardwareMap.dcMotor.get("liftMotor2");
    }

    public void updateRow(int index) {
        targetPos = rowHeights.get(index);

    }

    public void loop() {
        double controllerPower = controller.calculate(targetPos, read());
        setPower(Range.clip(controllerPower + F, -1, 1));

    }

    public void setPower(double power) {
        motor1.setPower(power);
        motor2.setPower(-power);
    }

    public double read() {
        return (motor2.getCurrentPosition());
    }

}
