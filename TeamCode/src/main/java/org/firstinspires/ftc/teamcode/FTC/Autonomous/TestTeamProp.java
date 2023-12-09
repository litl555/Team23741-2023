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
        TeamPropDetectionPipeline pipeline = new TeamPropDetectionPipeline(cam, telemetry1, true);
        waitForStart();

        boolean aDown = false;
        boolean lbDown = false;

        boolean isDroppingBottomPixel = true;

        while (opModeIsActive()) {
            telemetry1.add("Detected position", pipeline.propPos);
            telemetry1.update();
        }
    }
}
