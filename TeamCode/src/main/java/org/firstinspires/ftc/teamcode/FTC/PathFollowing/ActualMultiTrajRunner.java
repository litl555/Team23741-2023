package org.firstinspires.ftc.teamcode.FTC.PathFollowing;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

import java.util.HashMap;

public class ActualMultiTrajRunner {
    private Trajectory[] trajectories;
    private TrajectoryRunner[] trajectoryRunners;
    private SimpleTrajectory[] trajectoryMetadata;
    private HashMap<Integer, MultiTrajEvent> events = new HashMap<>();
    private boolean finished = false, started = false, paused = false;
    private int currentlyRunningIndex = 0;

    public ActualMultiTrajRunner() {

    }

    public ActualMultiTrajRunner(SimpleTrajectory[] traj) {

        initialize(traj);
    }

    // checkpoint system to allow us to run arbitrary commands once the robot completes a certain amount of trajectories
    // note that the scheduled checkpoint will run after the respective trajectory finishes
    // ie command at checkpoint 0 runs after the first (0 based index due to arrays) trajectory finishes
    // | tr 0 | cp 0 | tr 1 | cp 1 ...
    public ActualMultiTrajRunner(SimpleTrajectory[] traj, MultiTrajEvent[] events) {
        for (MultiTrajEvent event : events) this.events.put(event.checkpoint, event);
        initialize(traj);
    }

    public void addEvents(MultiTrajEvent[] events) {
        for (MultiTrajEvent event : events) this.events.put(event.checkpoint, event);
    }

    public void initEvents(SimpleTrajectory[] traj) {
        initialize(traj);
    }

    private void initialize(SimpleTrajectory[] traj) {
        trajectories = new Trajectory[traj.length];
        trajectoryRunners = new TrajectoryRunner[traj.length];

        for (int i = 0; i < traj.length; i++) {
            SimpleTrajectory st = traj[i];

            // true false | false false | false true
            boolean start = (i == 0) || events.containsKey(i - 1);
            boolean end = (i == traj.length - 1) || events.containsKey(i);
            Trajectory t = st.getTrajectory(start, end);

            trajectories[i] = t;
        }

        this.trajectoryMetadata = traj;
    }

    public void update() {
        if (finished || !started || paused) return;

        TrajectoryRunner currentTr = trajectoryRunners[currentlyRunningIndex];
        if (currentTr.currentState != TrajectoryRunner.State.PRESTART) currentTr.update();

        if (currentTr.currentState == TrajectoryRunner.State.FINISHED) {
            if (currentlyRunningIndex == trajectoryRunners.length - 1) {
                finished = true;
                Robot.isBusy = false;
            } else {
                currentTr.currentState = TrajectoryRunner.State.PRESTART;
                currentlyRunningIndex++;

                if (events.containsKey(currentlyRunningIndex - 1)) {
                    CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                            events.get(currentlyRunningIndex - 1).event,
                            new InstantCommand(this::notifyUnpause)));
                    paused = true;
                } else startTrajectoryRunner(currentlyRunningIndex);
            }
        }
    }

    public void start() {
        if (started || finished) return;

        Robot.isBusy = true;
        started = true;
        paused = false;
        startTrajectoryRunner(currentlyRunningIndex);
    }

    public boolean hasFinished() { return finished; }
    public boolean isRunning() { return started && !finished; }
    public boolean isPaused() { return paused; }
    public void notifyUnpause() {
        startTrajectoryRunner(currentlyRunningIndex);
        paused = false;
    }

    private void startTrajectoryRunner(int id) {
        trajectoryRunners[id] = trajectoryMetadata[id].getTrajectoryRunner(trajectories[id]);
        trajectoryRunners[id].start();
    }
}