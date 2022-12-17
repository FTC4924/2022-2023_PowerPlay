package org.firstinspires.ftc.teamcode.Commands.preftclib;

import androidx.annotation.FloatRange;

import static org.firstinspires.ftc.teamcode.Constants.CLAW_GRABBER_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.CLAW_GRABBER_OPEN_POSITION;

public class GripperGrab extends Command {
    /**
     * Open/close the gripper
     * @param percent The percent to open. 0 is closed, 1 is open.
     */
    public GripperGrab(@FloatRange(from = 0.0, to = 1.0) double percent) {
        this.distance = CLAW_GRABBER_CLOSE_POSITION + percent * (CLAW_GRABBER_OPEN_POSITION - CLAW_GRABBER_CLOSE_POSITION);
    }
}
