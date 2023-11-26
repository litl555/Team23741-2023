package org.firstinspires.ftc.teamcode.FTC.Pixels;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;
import org.firstinspires.ftc.teamcode.FTC.Pixels.Types.Hex;
import org.firstinspires.ftc.teamcode.FTC.Pixels.Types.PixelColor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.Map;

@TeleOp
public class TestAprilTag extends LinearOpMode {
    @Override
    public void runOpMode() {
        LoggerTool telemetry = new LoggerTool();

        OpenCvCamera cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "outtake_camera"));
        BoardDetectionPipeline pipeline = new BoardDetectionPipeline(cam, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            if (!pipeline.hasInitialized()) {
                telemetry.add("STATUS", "Pipeline still initializing");
            } else {
                if (gamepad1.x && pipeline.canRequestDetection()) {
                    pipeline.requestDetection(true);
                    telemetry.add("STATUS", "Requesting detection");
                }

                if (gamepad1.y) {
                    if (!pipeline.canRequestDetection()) {
                        telemetry.add("ERROR", "Pipeline is currently running, cannot request results");
                    } else {
                        telemetry.add("TAG ID", (pipeline.getTag().isRed ? "red" : "blue") + " " + pipeline.getTag().id);
                        Map<Hex, PixelColor> board = pipeline.getBoard();

                        int nwhite = 0, npurple = 0, ngreen = 0, nyellow = 0;

                        for (PixelColor color : board.values()) {
                            if (color == PixelColor.green) ngreen++;
                            else if (color == PixelColor.purple) npurple++;
                            else if (color == PixelColor.white) nwhite++;
                            else if (color == PixelColor.yellow) nyellow++;
                        }

                        telemetry.add("NUM WHITE", nwhite);
                        telemetry.add("NUM GREEN", ngreen);
                        telemetry.add("NUM YELLOW", nyellow);
                        telemetry.add("NUM PURPLE", npurple);
                    }
                }
            }

            telemetry.update();
        }
    }
}
