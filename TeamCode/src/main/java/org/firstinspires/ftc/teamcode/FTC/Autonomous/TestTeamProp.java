package org.firstinspires.ftc.teamcode.FTC.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Config
@TeleOp
public class TestTeamProp extends LinearOpMode {
    public static double liftPower = 0.25;
    public static double _ccl = 0.8, _ccr = 0.8;
    public static double _at = 0.5, _ab = 0.5;
    public static double _ct = 0.5, _cb = 0.5;

    @Override
    public void runOpMode() {
        LoggerTool telemetry1 = new LoggerTool(telemetry);

        OpenCvCamera cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "outtake_camera"));
        TeamPropDetectionPipeline pipeline = new TeamPropDetectionPipeline(cam, telemetry1, false);

        DcMotor lift1 = hardwareMap.get(DcMotor.class, "lift1");
        DcMotor lift2 = hardwareMap.get(DcMotor.class, "lift2");

        Servo ccl = hardwareMap.get(Servo.class, "clawCenterLeft"); // con 1
        Servo at = hardwareMap.get(Servo.class, "armTop"); // con 2
        Servo ab = hardwareMap.get(Servo.class, "armBottom"); // con 3

        Servo ccr = hardwareMap.get(Servo.class, "clawCenterRight"); // exp 2
        Servo ct = hardwareMap.get(Servo.class, "clawTop"); // exp 3
        Servo cb = hardwareMap.get(Servo.class, "clawBottom"); // exp 4

        /*
         * center (cc) : 0.7 -> 1 (open, close)
         * arm    (a)  :
         * claw   (c)  :
         */

        waitForStart();

        boolean aDown = false;
        boolean lbDown = false;

        boolean isDroppingBottomPixel = true;

        while (opModeIsActive()) {
            telemetry1.add("Detected position", pipeline.propPos);
            telemetry1.update();

            // when done call
            // pipeline.destroy();

            if (gamepad1.dpad_up) {
                lift1.setPower(liftPower);
                lift2.setPower(liftPower);
            } else if (gamepad1.dpad_down) {
                lift1.setPower(-1 * liftPower);
                lift2.setPower(-1 * liftPower);
            } else {
                lift1.setPower(0);
                lift2.setPower(0);
            }

            // close
            if (gamepad1.right_bumper) {
                ccl.setPosition(1);
                ccr.setPosition(1);

                isDroppingBottomPixel = true;
            }

            // open
            if (gamepad1.left_bumper && !lbDown) {
                lbDown = true;

                if (isDroppingBottomPixel) ccl.setPosition(0.7);
                else ccr.setPosition(0.7);

                isDroppingBottomPixel = !isDroppingBottomPixel;
            } else if (!gamepad1.left_bumper) lbDown = false;

            if (gamepad1.a && !aDown) {
                aDown = true;

                ccl.setPosition(_ccl);
                ccr.setPosition(_ccr);
                at.setPosition(_at);
                ab.setPosition(_ab);
                ct.setPosition(_ct);
                cb.setPosition(_cb);
            } else if (!gamepad1.right_bumper) aDown = false;
        }
    }
}
