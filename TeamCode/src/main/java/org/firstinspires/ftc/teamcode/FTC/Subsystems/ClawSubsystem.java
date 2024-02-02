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
        new ArmWristPos(0.01, -0.007),
        new ArmWristPos(0.01, -0.007),
        new ArmWristPos(-0.03055, 0.043),
    // ----
        new ArmWristPos(-0.07777, 0.083333), // --- 0.083333
        new ArmWristPos(-0.08833, 0.083333),
        new ArmWristPos(-0.07777, 0.083333),
        new ArmWristPos(-0.08833, 0.083333),
        new ArmWristPos(-0.07777, 0.083333),
        new ArmWristPos(-0.08833, 0.083333),
        new ArmWristPos(-0.08833, 0.083333),
        new ArmWristPos(-0.08833, 0.083333)
    };

    public static final ArmWristPos hangDelta = new ArmWristPos(-0.1549999, 0.205);
    public static final ArmWristPos clearPixelIntake = new ArmWristPos(0, -0.02055);
    //public static final ArmWristPos zero = new ArmWristPos(0.5777777, 0.3577777);
    public static final ArmWristPos zero = new ArmWristPos(0.5777777 + 0.02, 0.33666);
    public static final ArmWristPos pickZero = new ArmWristPos(0, 0.02);

    public ClawState currentState = ClawState.UNKNOWN;

    public static final double closedPosTop = 1.0, closedPosBot = 1.0, halfPos = .85, openPos = .7;
    public void update(ClawState state) {
        currentState = state;
        Robot.hardware.setClaw(state);
    }

    public void updateArmWristPos(int index) { rowDelta[index].apply(this); }

    public void setWrist(double angle) { Robot.hardware.setWrist(angle); }

    public void setArm(double angle) { Robot.hardware.setArm(angle); }

    public void enableHang() { hangDelta.apply(this); }

    public enum ClawState {
        CLOSED,
        OPENONE,
        OPEN,
        HALFCLOSE,
        UNKNOWN
    }
}
