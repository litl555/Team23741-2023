package org.firstinspires.ftc.teamcode.FTC.PathFollowing;

import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

import java.util.ArrayList;

public class MultipleTrajectoryRunner {
    public int currentT = 0;
    ArrayList<TrajectoryRunner> trajectoryRunners;
    public boolean finished = false;

    public MultipleTrajectoryRunner(ArrayList<TrajectoryRunner> trajectoryRunners) {
        this.trajectoryRunners = trajectoryRunners;


    }

    public void start() {
        trajectoryRunners.get(currentT).start();
        Robot.isBusy = true;
    }

    public void update() {
        if (trajectoryRunners.get(currentT).currentState != TrajectoryRunner.State.FINISHED && trajectoryRunners.get(currentT).currentState != TrajectoryRunner.State.PRESTART) {
            trajectoryRunners.get(currentT).update();
        } else if (trajectoryRunners.get(currentT).currentState == TrajectoryRunner.State.FINISHED || trajectoryRunners.get(currentT).currentState == TrajectoryRunner.State.PRESTART) {
            if (currentT + 1 < trajectoryRunners.size()) {
                trajectoryRunners.get(currentT + 1).start();

                trajectoryRunners.get(currentT).currentState = TrajectoryRunner.State.PRESTART;
                currentT++;
            } else {
                trajectoryRunners.get(currentT).currentState = TrajectoryRunner.State.PRESTART;
                finished = true;
                Robot.isBusy = false;
            }

        }

    }
}
