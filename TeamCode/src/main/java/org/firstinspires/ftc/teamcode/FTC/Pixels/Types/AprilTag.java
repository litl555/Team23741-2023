package org.firstinspires.ftc.teamcode.FTC.Pixels.Types;

import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.FTC.Pixels.Constants.BoardConstants;

public class AprilTag {
    public Pose pose;
    public Pose2d positionFromCenter; // in meters
    public int id;
    public boolean isRed;

    public AprilTag(Pose pose, int id) {
        this.pose = pose;
        this.id = id;

        isRed = id > 3; // blue is 1-3, red is 4-6

        // +x is left
        // +y is down
        // +z is towards camera

        Pose2d ref = BoardConstants.tagID.positionFromCenter.get(id);
        positionFromCenter = new Pose2d(
                ref.getX() - pose.tvec.get(0, 0)[0], // x
                ref.getY() - pose.tvec.get(2, 0)[0], // z
                ref.getRotation()
        );
    }
}
