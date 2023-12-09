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

import org.firstinspires.ftc.teamcode.FTC.TeleOp.ArmWristPos;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ClawSubsystem extends SubsystemBase {
    // this matches LiftSubsystem values
    // 0 -> intake
    // 1 -> slightly above pixel, in tray
    // 2 -> ground
    // 3 -> board row 1
    // 4 -> board row 2
    // 5 -> board row 3...
    public static ArmWristPos[] rowDelta = new ArmWristPos[] {
            new ArmWristPos(0.02122, -0.0338888),
            new ArmWristPos(0.02122, -0.0338888),
            new ArmWristPos(-0.00611, 0.03611),
            new ArmWristPos(-0.07777, 0.066111),
            new ArmWristPos(-0.09833, 0.066111),
            new ArmWristPos(-0.06777, 0.066111),
            new ArmWristPos(-0.08777, 0.066111),
            new ArmWristPos(-0.03777, 0.066111),
            new ArmWristPos(-0.07777, 0.066111),
    };

    public static ArmWristPos clearPixelIntake = new ArmWristPos(-0.01888, -0.075);
    public ClawState currentState = ClawState.UNKNOWN;


    // TODO: remove
    public List<Double> rowWrist = Arrays.asList(wristIntake, wristPlacing, wristClearing, wristGround);
    public List<Double> rowArm = Arrays.asList(armIntake, armPlace1, armPlace2, armGround);
    private double closedPosTop = 1.0, closedPosBot = 1.0, halfPos = .85, openPos = .67;

    // zero is the starting position of claw
    public static ArmWristPos zero = new ArmWristPos(0.477777, 0.4638887);


    public void update(ClawState state) {
        currentState = state;

        switch (state) {
            case OPEN:
                Robot.clawBottom.setPosition(openPos);
                Robot.clawTop.setPosition(openPos);
                break;
            case CLOSED:
                Robot.clawTop.setPosition(closedPosTop);
                Robot.clawBottom.setPosition(closedPosBot);
                break;
            case OPENONE:
                Robot.clawBottom.setPosition(openPos);
                Robot.clawTop.setPosition(closedPosBot);
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

    public void updateArmWristPos(int index) { rowDelta[index].apply(this); }

    public void setWrist(double angle) {
        Robot.wrist1.setPosition(angle);
        Robot.wrist2.setPosition(angle);
    }

    public void setArm(double angle) {
        Robot.arm1.setPosition(angle);
        Robot.arm2.setPosition(angle);
    }

    public void syncRows(int index) {
        updateWristRow(index);
        updateArmRow(index);
    }

    public enum ClawState {
        CLOSED,
        OPENONE,
        OPEN,
        HALFCLOSE,
        UNKNOWN
    }
}
