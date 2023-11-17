package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.GoToPosPID;

/**
 * Used to go to position without motion profile for basic movements. Hacky but effective.
 **/
public class PIDToPose extends CommandBase {
    Pose2d end;
    GoToPosPID go;
    CustomLocalization l;

    public PIDToPose(Pose2d end, CustomLocalization l) {
        this.end = end;
        this.l = l;
    }

    @Override
    public void initialize() {
        go = new GoToPosPID(end, l);
    }

    @Override
    public void execute() {
        go.update();

    }

    @Override
    public boolean isFinished() {
        return (go.isFinished());
    }
}
