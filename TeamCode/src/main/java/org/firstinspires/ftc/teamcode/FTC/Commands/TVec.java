package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.Pixels.Types.Pose;

public class TVec {
    double x, y, z;
    Pose2d cameraPos = new Pose2d(180, 180);
    Pose2d boardPos1 = new Pose2d(900 - 25.4 * 6, 1500);
    Pose2d boardPos2 = new Pose2d(900, 1500);
    Pose2d boardPos3 = new Pose2d(900 + 25.4 * 6, 1500);
    public static Pose2d worldPos = new Pose2d();

    public void updateTvec(double[] tvec, int pos) {
        tvec = addRobot(tvec);
        tvec = rotateByRobotAngle(tvec);
        if (pos == 1) {
            worldPos = boardPos1.plus((new Pose2d(tvec[0], tvec[1])).times(-1.0));
        } else if (pos == 2) {
            worldPos = boardPos2.plus((new Pose2d(tvec[0], tvec[1])).times(-1.0));
        } else if (pos == 3) {
            worldPos = boardPos3.plus((new Pose2d(tvec[0], tvec[1])).times(-1.0));
        } else {
            worldPos = boardPos2.plus((new Pose2d(tvec[0], tvec[1])).times(-1.0));
        }


    }

    private double[] addRobot(double[] tvec) {
        double[] robotVec = new double[]{cameraPos.getX() + tvec[0], -tvec[2] + cameraPos.getY()};
        return robotVec;
    }

    private double[] rotateByRobotAngle(double[] tvec) {
        return (new double[]{tvec[0] * Math.cos(Constants.angle) - tvec[1] * Math.sin(Constants.angle), tvec[0] * Math.sin(Constants.angle) + tvec[1] * Math.cos(Constants.angle)});
    }
}
