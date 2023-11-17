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

    public List<Double> rowWrist = Arrays.asList(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    public List<Double> rowArm = Arrays.asList(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    private double closedPos1 = 0.0;
    private double closedPos2 = 0.0;
    private double halfPos = .5;
    private double openPos = 1.0;

    private double gearRatio = 2.0 / 3.0;
    private double maxDegrees = 45;

    public ClawSubsystem() {

    }

    public void update(ClawState state) {
        switch (state) {
            case OPEN:
                Robot.claw1.setPosition(openPos);
                Robot.claw2.setPosition(openPos);
            case CLOSED:
                Robot.claw1.setPosition(closedPos1);
                Robot.claw2.setPosition(closedPos2);
            case OPENONE:
                Robot.claw1.setPosition(openPos);
                Robot.claw2.setPosition(closedPos2);
            case HALFCLOSE:
                Robot.claw2.setPosition(halfPos);
                Robot.claw1.setPosition(halfPos);
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
        return (angle / (300 * gearRatio) + (maxDegrees - (300 * gearRatio)) / (300 * gearRatio));
    }
}
