package org.firstinspires.ftc.teamcode.localiation;

import java.util.ArrayList;

public class MultipleTrajectoryRunner {
    int currentT = 0;
    ArrayList<TrajectoryRunner> trajectoryRunners;
    public boolean finished = false;

    public MultipleTrajectoryRunner(ArrayList<TrajectoryRunner> trajectoryRunners) {
        this.trajectoryRunners = trajectoryRunners;
    }

    public void start() {
        trajectoryRunners.get(currentT).start();
    }

    public void update() {
        if (trajectoryRunners.get(currentT).currentState != TrajectoryRunner.State.FINISHED && trajectoryRunners.get(currentT).currentState != TrajectoryRunner.State.PRESTART) {
            trajectoryRunners.get(currentT).update();
        } else if (trajectoryRunners.get(currentT).currentState == TrajectoryRunner.State.FINISHED) {
            if (currentT + 1 < trajectoryRunners.size()) {
                trajectoryRunners.get(currentT + 1).start();
                trajectoryRunners.get(currentT).currentState = TrajectoryRunner.State.PRESTART;
                currentT++;
            } else {
                trajectoryRunners.get(currentT).currentState = TrajectoryRunner.State.PRESTART;
                finished = true;
            }

        }

    }
}
