package org.firstinspires.ftc.teamcode.FTC.Localization;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class OdometryModule {
    private final DcMotor encoder;
    private int ticks, ticksTemp, currentTicks;
    public double tickDeltaMm;

    private int offset = 1;

    public OdometryModule(DcMotor encoder) {
        this.encoder = encoder;
        encoder.setDirection((DcMotorSimple.Direction.FORWARD));

        this.ticks = 0;
    }

    public void reverse() {
        //encoder.setDirection(DcMotorSimple.Direction.REVERSE);
        offset = -1;
    }

    public void getDelta() {
        currentTicks = offset * encoder.getCurrentPosition();
        ticksTemp = ticks;
        ticks = currentTicks;
        tickDeltaMm = Constants.ticksToMM(currentTicks - ticksTemp);
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
