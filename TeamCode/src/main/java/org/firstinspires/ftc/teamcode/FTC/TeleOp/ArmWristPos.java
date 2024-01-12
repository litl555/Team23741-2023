package org.firstinspires.ftc.teamcode.FTC.TeleOp;

import org.firstinspires.ftc.teamcode.FTC.Subsystems.ClawSubsystem;

public class ArmWristPos {
    public final double arm, wrist;
    public ArmWristPos(double arm, double wrist) {
        this.arm = arm;
        this.wrist = wrist;
    }

    public void apply(ClawSubsystem claw) {
        claw.setArm(arm + ClawSubsystem.zero.arm);
        claw.setWrist(wrist + ClawSubsystem.zero.wrist);
    }

    public void override(ClawSubsystem claw) {
        claw.setArm(arm);
        claw.setWrist(wrist);
    }
}
