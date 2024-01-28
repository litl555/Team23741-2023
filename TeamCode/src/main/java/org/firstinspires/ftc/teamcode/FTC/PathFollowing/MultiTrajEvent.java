package org.firstinspires.ftc.teamcode.FTC.PathFollowing;

import com.arcrobotics.ftclib.command.Command;

public class MultiTrajEvent {
    public final int checkpoint;
    public final Command event;
    public final boolean pause;

    public MultiTrajEvent(int checkpoint, Command event, boolean pause) {
        this.pause = pause;
        this.checkpoint = checkpoint;
        this.event = event;
    }
}
