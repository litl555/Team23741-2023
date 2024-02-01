package org.firstinspires.ftc.teamcode.FTC.PathFollowing;

import com.arcrobotics.ftclib.command.Command;

public class MultiTrajEvent {
    public final int checkpoint;
    public final Command event;

    public MultiTrajEvent(int checkpoint, Command event) {
        this.checkpoint = checkpoint;
        this.event = event;
    }
}
