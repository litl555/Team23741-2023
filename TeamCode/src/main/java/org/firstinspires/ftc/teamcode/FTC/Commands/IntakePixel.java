package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.FTC.Subsystems.IntakeSubsystem;

public class IntakePixel extends CommandBase {
    private boolean finished = false;
    IntakeSubsystem intake;
    private double intakeMMThreshold = 20.0;

    public IntakePixel(IntakeSubsystem intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.update(IntakeSubsystem.IntakePowerSetting.INTAKE);
    }

    @Override
    public void execute() {
        if (intake.getDistance() > intakeMMThreshold) {
            if (intake.seePixel == true) {
                intake.pixelPassCount++;

            }
            intake.seePixel = false;
        } else {
            intake.seePixel = true;
        }
        if (intake.pixelPassCount > 1) {
            finished = true;
            intake.update(IntakeSubsystem.IntakePowerSetting.OUTTAKE);
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
