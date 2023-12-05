package org.firstinspires.ftc.teamcode.FTC.Subsystems;

import static org.firstinspires.ftc.teamcode.FTC.TeleOp.TeleOpConstants.armGround;
import static org.firstinspires.ftc.teamcode.FTC.TeleOp.TeleOpConstants.armIntake;
import static org.firstinspires.ftc.teamcode.FTC.TeleOp.TeleOpConstants.armPlace1;
import static org.firstinspires.ftc.teamcode.FTC.TeleOp.TeleOpConstants.armPlace2;
import static org.firstinspires.ftc.teamcode.FTC.TeleOp.TeleOpConstants.wristClearing;
import static org.firstinspires.ftc.teamcode.FTC.TeleOp.TeleOpConstants.wristGround;
import static org.firstinspires.ftc.teamcode.FTC.TeleOp.TeleOpConstants.wristIntake;
import static org.firstinspires.ftc.teamcode.FTC.TeleOp.TeleOpConstants.wristPlacing;

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

    public List<Double> rowWrist = Arrays.asList(wristIntake, wristPlacing, wristClearing, wristGround);
    public List<Double> rowArm = Arrays.asList(armIntake, armPlace1, armPlace2, armGround);
    private double closedPos1 = 1.0;
    private double closedPos2 = 1.0;
    private double halfPos = .85;
    private double openPos = .67;

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

    public void updateWrist(double change) {
        Robot.wrist1.setPosition(Robot.wrist1.getPosition() + change);
        Robot.wrist2.setPosition(Robot.wrist2.getPosition() + change);
    }

    public void setArm(double angle) {
        Robot.arm1.setPosition(angleToServo(angle));
        Robot.arm2.setPosition(angleToServo(angle));
    }

    public void updateArm(double change) {
        Robot.arm1.setPosition(Robot.arm1.getPosition() + change);
        Robot.arm2.setPosition(Robot.arm2.getPosition() + change);
    }

    public void syncRows(int index) {
        updateWristRow(index);
        updateArmRow(index);
    }

    private double angleToServo(double angle) {
        return (angle);
//        return (angle / (300 * gearRatio) + (maxDegrees - (300 * gearRatio)) / (300 * gearRatio));
    }
}
