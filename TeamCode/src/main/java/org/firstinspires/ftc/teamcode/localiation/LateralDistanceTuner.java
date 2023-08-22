package org.firstinspires.ftc.teamcode.localiation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.DashboardUtil;
@TeleOp
@Config
public class LateralDistanceTuner extends LinearOpMode {

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {
        CustomLocalization localizer=new CustomLocalization(new Pose2d(0,0,0),hardwareMap);
        waitForStart();
        while(opModeIsActive()&&!isStopRequested()){

            boolean stop=false;
            while(!stop){
                TelemetryPacket packet=new TelemetryPacket();

                if(gamepad1.y){
                    stop=true;
                }
                localizer.setWeightedDrivePowers(new Pose2d(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x));
                localizer.updateMethod();
                DashboardUtil.drawRobot(packet.fieldOverlay(),new Pose2d(Constants.robotPose.getX()*.0394,Constants.robotPose.getY()*.0394,(Constants.robotPose.getHeading())));
                dashboard.sendTelemetryPacket(packet);
            }
            localizer.setMotorPowers(0,0,0,0);
            telemetry.addData("Lateral Distance: ",(double)((Constants.angle)/((double)10*Math.PI*(double)2)*Constants.LATERAL_DISTANCE));
            telemetry.update();
            while(!isStopRequested()){
                idle();
            }
        }
    }
}
