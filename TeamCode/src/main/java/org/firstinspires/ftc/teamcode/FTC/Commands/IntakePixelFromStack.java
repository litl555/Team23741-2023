package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.FTC.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

public class IntakePixelFromStack extends CommandBase {
    // assumptions:
    // we are directly in front of the stack, intake facing the stack
    private int numToCollect, collected, initialPixelCount;
    private long maxTime, startTime;
    private boolean finished = false;

    private double increment = 0.01, initialLevel;
    private int incrementLevel = 0;
    private long incrementTime = 250;

    // time in milliseconds, starting level 1 based (1 is first pixel)
    public IntakePixelFromStack(int numToCollect, long maxTime, int startingLevel) {
        this.numToCollect = numToCollect;
        this.maxTime = maxTime;
        initialLevel = IntakeSubsystem.droptakeLevel[startingLevel] + 0.02;
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        initialPixelCount = Robot.intakeSubsystem.pixelPassCount;

        Robot.intakeSubsystem.setDroptakePosition(IntakeSubsystem.droptakeLevel[5]);
        Robot.intakeSubsystem.setPower(-.8);

        Robot.intakeSubsystem.activateIntakeDist.set(true);
    }

    @Override
    public void execute() {
        if (finished) return;
        long t = System.currentTimeMillis();

        collected = Robot.intakeSubsystem.pixelPassCount - initialPixelCount;
        finished = collected == numToCollect || t - startTime >= maxTime;

        // cleanup should be handled by main thread logic
        if (finished) {
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
