package org.firstinspires.ftc.teamcode.FTC.Pixels;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;
import org.firstinspires.ftc.teamcode.FTC.Pixels.Constants.BoardConstants;
import org.firstinspires.ftc.teamcode.FTC.Pixels.Types.Hex;
import org.firstinspires.ftc.teamcode.FTC.Pixels.Types.PixelColor;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Map;

@Config
@TeleOp
public class TestAprilTag extends LinearOpMode {
    public static double brightness;

    @Override
    public void runOpMode() throws InterruptedException {
        LoggerTool telemetry = new LoggerTool(this.telemetry);

        OpenCvCamera cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "outtake_camera"));
        BoardDetectionPipeline pipeline = new BoardDetectionPipeline(cam, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            pipeline.brightness = brightness;
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

                        telemetry.add("DIST X", pipeline.getTag().positionFromCenter.getX() * BoardConstants.meterToInch);
                        telemetry.add("DIST Y", pipeline.getTag().positionFromCenter.getY() * BoardConstants.meterToInch);

                        telemetry.add("TVEC X", pipeline.getTag().pose.tvec.get(0, 0)[0] * BoardConstants.meterToInch);
                        telemetry.add("TVEC Y", pipeline.getTag().pose.tvec.get(1, 0)[0] * BoardConstants.meterToInch);
                        telemetry.add("TVEC Z", pipeline.getTag().pose.tvec.get(2, 0)[0] * BoardConstants.meterToInch);
                    }
                }
            }

            telemetry.add("MEMORY FREE", (double) Runtime.getRuntime().freeMemory() / (double) Runtime.getRuntime().totalMemory());

            telemetry.update();

            sleep(20);
        }
    }
}
