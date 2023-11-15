package org.firstinspires.ftc.teamcode.localiation.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ClawTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo claw = hardwareMap.servo.get("claw");
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.a) {
                claw.setPosition(.333);

            } else if (gamepad1.b) {
                claw.setPosition(0);
            }
        }

    }
}
