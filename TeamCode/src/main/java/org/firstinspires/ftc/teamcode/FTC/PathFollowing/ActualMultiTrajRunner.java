package org.firstinspires.ftc.teamcode.FTC.PathFollowing;

public class ActualMultiTrajRunner {
    private Trajectory[] trajectories;
    private TrajectoryRunner[] trajectoryRunners;
    private boolean finished = false, started = false;
    private int currentlyRunningIndex = 0;

    public ActualMultiTrajRunner(SimpleTrajectory[] traj) {
        trajectories = new Trajectory[traj.length];
        trajectoryRunners = new TrajectoryRunner[traj.length];

        for (int i = 0; i < traj.length; i++) {
            SimpleTrajectory st = traj[i];

            // true false | false false | false true
            Trajectory t = st.getTrajectory(i == 0, i == traj.length - 1);
            TrajectoryRunner tr = st.getTrajectoryRunner(t);

            trajectories[i] = t;
            trajectoryRunners[i] = tr;
        }
    }

    public void update() {
        if (finished || !started) return;

        TrajectoryRunner currentTr = trajectoryRunners[currentlyRunningIndex];
        currentTr.update();

        if (currentTr.currentState == TrajectoryRunner.State.FINISHED) {
            if (currentlyRunningIndex == trajectoryRunners.length - 1) {
                finished = true;
                return;
            } else {
                currentTr.currentState = TrajectoryRunner.State.PRESTART;
                currentlyRunningIndex++;

                trajectoryRunners[currentlyRunningIndex].start();
            }
        }
    }

    public void start() {
        if (started || finished) return;

        started = true;
        trajectoryRunners[currentlyRunningIndex].start();
    }

    public boolean hasFinished() { return finished; }
    public boolean isRunning() { return started && !finished; }
}
