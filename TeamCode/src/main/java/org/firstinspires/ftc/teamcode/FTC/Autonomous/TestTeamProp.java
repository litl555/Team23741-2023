package org.firstinspires.ftc.teamcode.FTC.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Config
@TeleOp
public class TestTeamProp extends LinearOpMode {

    @Override
    public void runOpMode() {
        LoggerTool telemetry1 = new LoggerTool(telemetry);

        OpenCvCamera cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "outtake_camera"));
        TeamPropDetectionPipeline pipeline = new TeamPropDetectionPipeline(cam, telemetry1, false);

        waitForStart();

        while (opModeIsActive()) {
            telemetry1.add("Detected position", pipeline.propPos);
            telemetry1.update();

            // when done call
            // pipeline.destroy();
        }
    }
}
