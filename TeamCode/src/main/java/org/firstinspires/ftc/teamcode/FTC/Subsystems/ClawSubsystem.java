package org.firstinspires.ftc.teamcode.FTC.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.FTC.TeleOp.ArmWristPos;

public class ClawSubsystem extends SubsystemBase {
    // this matches LiftSubsystem values
    // 0 -> intake
    // 1 -> slightly above pixel, in tray
    // 2 -> ground
    // 3 -> board row 1
    // 4 -> board row 2
    // 5 -> board row 3...
    private static final ArmWristPos[] rowDelta = new ArmWristPos[] {
            new ArmWristPos(0, -0.010555),
            new ArmWristPos(0, -0.010555),
            new ArmWristPos(-0.03055, 0.043),
        // ----
            new ArmWristPos(-0.07777, 0.083333), // --- 0.083333
            new ArmWristPos(-0.08833, 0.083333),
            new ArmWristPos(-0.07777, 0.083333),
            new ArmWristPos(-0.08833, 0.083333),
            new ArmWristPos(-0.07777, 0.083333),
            new ArmWristPos(-0.08833, 0.083333),
    };

    public static final ArmWristPos hangDelta = new ArmWristPos(-0.1549999, 0.205);
    public static final ArmWristPos clearPixelIntake = new ArmWristPos(0, -0.02055);
    //public static final ArmWristPos zero = new ArmWristPos(0.5777777, 0.3577777);
    public static final ArmWristPos zero = new ArmWristPos(0.5777777, 0.33666);

    public ClawState currentState = ClawState.UNKNOWN;

    private final double closedPosTop = 1.0, closedPosBot = 1.0, halfPos = .85, openPos = .7;
    public void update(ClawState state) {
        currentState = state;

        switch (state) {
            case OPEN:
                Robot.clawBlack.setPosition(openPos);
                Robot.clawWhite.setPosition(openPos);
                break;
            case CLOSED:
                Robot.clawWhite.setPosition(closedPosTop);
                Robot.clawBlack.setPosition(closedPosBot);
                break;
            case OPENONE:
                Robot.clawBlack.setPosition(openPos);
                Robot.clawWhite.setPosition(closedPosBot);
                break;
            case HALFCLOSE:
                Robot.clawWhite.setPosition(halfPos);
                Robot.clawBlack.setPosition(halfPos);
                break;
        }

    }

    public void updateArmWristPos(int index) { rowDelta[index].apply(this); }

    public void setWrist(double angle) {
        Robot.wristRed.setPosition(angle);
        Robot.wristBlue.setPosition(angle);
    }

    public void setArm(double angle) {
        Robot.armYellow.setPosition(angle);
        Robot.armGreen.setPosition(angle);
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
