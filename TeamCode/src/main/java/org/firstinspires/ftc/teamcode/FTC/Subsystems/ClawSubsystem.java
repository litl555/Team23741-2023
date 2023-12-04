package org.firstinspires.ftc.teamcode.FTC.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ClawSubsystem extends SubsystemBase {

    public enum ClawState {
        CLOSED,
        OPENONE,
        OPEN,
        HALFCLOSE
    }

    public static double wristIntake = 0.0;
    public static double wristClearing = 0.0;
    public static double wristGround = 0.0;
    public static double wristPlacing = 0.0;
    public List<Double> rowWrist = Arrays.asList(wristIntake, wristGround, wristPlacing, wristPlacing, wristPlacing, wristPlacing, wristPlacing);
    public List<Double> rowArm = Arrays.asList(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    private double closedPos1 = 1.0;
    private double closedPos2 = 1.0;
    private double halfPos = .85;
    private double openPos = .7;

    private double gearRatio = 2.0 / 3.0;
    private double maxDegrees = 45;

    public ClawSubsystem() {

    }

    public void update(ClawState state) {
        switch (state) {
            case OPEN:
                Robot.clawBottom.setPosition(openPos);
                Robot.clawTop.setPosition(openPos);
                break;
            case CLOSED:
                Robot.clawTop.setPosition(closedPos1);
                Robot.clawBottom.setPosition(closedPos2);
                break;
            case OPENONE:
                Robot.clawBottom.setPosition(openPos);
                Robot.clawTop.setPosition(closedPos2);
                break;
            case HALFCLOSE:
                Robot.clawTop.setPosition(halfPos);
                Robot.clawBottom.setPosition(halfPos);
                break;
        }

    }

    public void updateArmRow(int index) {
        setArm(rowArm.get(index));
    }

    public void updateWristRow(int index) {
        setWrist(rowWrist.get(index));
    }

    public void setWrist(double angle) {
        Robot.wrist1.setPosition(angleToServo(angle));
        Robot.wrist2.setPosition(angleToServo(angle));
    }

    public void setArm(double angle) {
        Robot.arm1.setPosition(angleToServo(angle));
        Robot.arm2.setPosition(angleToServo(angle));
    }

    private double angleToServo(double angle) {
        return (angle);
//        return (angle / (300 * gearRatio) + (maxDegrees - (300 * gearRatio)) / (300 * gearRatio));
    }
}
