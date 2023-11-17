package org.firstinspires.ftc.teamcode.FTC.PathFollowing;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;

public class GoToPosPID {
    Pose2d end;
    CustomLocalization l;
    int inCount = 0;
    boolean finished = false;
    BasicPID controllerx = new BasicPID(new PIDCoefficients(FollowerConstants.kpxy, 0, 0));
    BasicPID controllery = new BasicPID(new PIDCoefficients(FollowerConstants.kpxy, 0, 0));

    public GoToPosPID(Pose2d end, CustomLocalization l) {
        this.end = end;
        this.l = l;

    }

    public void update() {
        if (!finished) {
            l.setWeightedDrivePowers(new Pose2d(controllerx.calculate(end.getX(), -Constants.robotPose.getY()), -controllery.calculate(end.getY(), Constants.robotPose.getX())));
            if (end.vec().distTo(new Pose2d(-Constants.robotPose.getY(), Constants.robotPose.getX(), 0).vec()) > 100) {
                inCount = 0;
            } else {
                inCount++;
            }
            if (inCount > 10) {
                finished = true;
            }
        }

    }

    public boolean isFinished() {
        return finished;
    }


}
