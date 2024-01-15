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
    private static final ArmWristPos[] rowDelta = new ArmWristPos[] {
            new ArmWristPos(0.025, -0.0070555),
            new ArmWristPos(0.025, -0.0070555),
            new ArmWristPos(-0.02055, 0.039444),
        // ----
            new ArmWristPos(-0.07777, 0.066111),
            new ArmWristPos(-0.08833, 0.056111),
            new ArmWristPos(-0.04777, 0.056111),
            new ArmWristPos(-0.08833, 0.056111),
            new ArmWristPos(-0.04777, 0.056111),
            new ArmWristPos(-0.04777, 0.056111),
    };

    public static final ArmWristPos hangDelta = new ArmWristPos(-0.1549999, 0.205);
    public static final ArmWristPos clearPixelIntake = new ArmWristPos(0.03, 0);
    public static final ArmWristPos zero = new ArmWristPos(0.5777777, 0.3577777);

    public ClawState currentState = ClawState.UNKNOWN;

    private final double closedPosTop = 1.0, closedPosBot = 1.0, halfPos = .85, openPos = .7;
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

    public void updateArmWristPos(int index) { rowDelta[index].apply(this); }

    public void setWrist(double angle) {
        Robot.wrist1.setPosition(angle);
        Robot.wrist2.setPosition(angle);
    }

    public void setArm(double angle) {
        Robot.arm1.setPosition(angle);
        Robot.arm2.setPosition(angle);
    }

    public void enableHang() { hangDelta.apply(this); }

    public enum ClawState {
        CLOSED,
        OPENONE,
        OPEN,
        HALFCLOSE,
        UNKNOWN
    }
}
