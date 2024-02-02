package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

public class IntakePixelFromStack extends CommandBase {
    // assumptions:
    // we are directly in front of the stack, intake facing the stack
    private int numToCollect, collected, initialPixelCount;
    private long maxTime, startTime;
    private boolean finished = false;

    private double increment = 0.01, initialLevel, intakePower;
    private int incrementLevel = 0;
    private long incrementTime = 500;

    // time in milliseconds, starting level 1 based (1 is first pixel)
    public IntakePixelFromStack(int numToCollect, long maxTime, int startingLevel) {
        this.numToCollect = numToCollect;
        this.maxTime = maxTime;
        initialLevel = IntakeSubsystem.droptakeLevel[startingLevel] + 0.02;
        intakePower = -0.8;
    }

    public IntakePixelFromStack(int numToCollect, long maxTime, int startingLevel, double intakePower) {
        this.numToCollect = numToCollect;
        this.maxTime = maxTime;
        initialLevel = IntakeSubsystem.droptakeLevel[startingLevel] + 0.02;
        this.intakePower = intakePower;
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        initialPixelCount = Robot.intakeSubsystem.pixelPassCount;

        Robot.intakeSubsystem.setDroptakePosition(initialLevel);
        Robot.intakeSubsystem.setPower(intakePower);

        Robot.intakeSubsystem.activateIntakeDist.set(true);
    }

    @Override
    public void execute() {
        if (finished) return;
        long t = System.currentTimeMillis();

        // note that this is for power = 1
        // normal operation (no blockage) is around 3-4k
        // full stop is 7k low power, 10k max power (batteru, ~12.3v to 13.5v)
        // note that it is difficult to detect if were on ground intaking because there is overlap between low and high battery vol
        double current = Robot.intakeMotor.getCurrent(CurrentUnit.MILLIAMPS);
        double scaledCurrent = Math.abs(current * (1.0 / intakePower));

        // note that this is for max power (13.23 is what this was tested at)
        if (scaledCurrent > 8250 * (Robot.startingBatteryVoltage / 13.23)) { // we are too close, back up
            Robot.customLocalization.setWeightedDrivePowers(new Pose2d(0, 0.4, 0));
        } else Robot.customLocalization.setWeightedDrivePowers(new Pose2d(0, -0.15, 0)); // otherwise move forwards

        collected = Robot.intakeSubsystem.pixelPassCount - initialPixelCount;
        finished = collected == numToCollect || t - startTime >= maxTime;

        // cleanup should be handled by main thread logic
        if (finished) {
            Robot.customLocalization.setWeightedDrivePowers(new Pose2d());
            Robot.intakeSubsystem.activateIntakeDist.set(false);
            Robot.intakeSubsystem.setDroptakePosition(IntakeSubsystem.droptakeLevel[IntakeSubsystem.droptakeLevel.length - 1]);
            Robot.intakeSubsystem.setPower(1);
        } else {
            long delta = t - startTime;
            if (delta / incrementTime > incrementLevel) {
                incrementLevel++;
                Robot.intakeSubsystem.setDroptakePosition(Math.max(0.02, initialLevel - (double) incrementLevel * increment));
            }
        }
    }

    @Override
    public boolean isFinished() { return finished; }
}
