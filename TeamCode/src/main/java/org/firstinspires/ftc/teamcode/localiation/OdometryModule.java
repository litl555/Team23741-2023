package org.firstinspires.ftc.teamcode.localiation;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class OdometryModule {
    private final DcMotor encoder;
    private int ticks;
    int currentTicks;

    public OdometryModule(DcMotor encoder) {
        this.encoder = encoder;

        this.ticks = 0;
    }

    public void reverse() {
        encoder.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double getDelta() {
        currentTicks = encoder.getCurrentPosition();

        return (Constants.ticksToMM(currentTicks - ticks));

    }

    public void update() {
        ticks = currentTicks;
    }

    public long getPosition() {
        return (ticks);
    }

    public void reset() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
