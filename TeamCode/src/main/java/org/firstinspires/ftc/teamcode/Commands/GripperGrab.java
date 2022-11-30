package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Constants.CLAW_GRABBER_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.CLAW_GRABBER_OPEN_POSITION;

public class GripperGrab extends Command {
    public GripperGrab(double percent) {
        this.position = (int)(CLAW_GRABBER_CLOSE_POSITION + percent * (CLAW_GRABBER_OPEN_POSITION - CLAW_GRABBER_CLOSE_POSITION));
    }
}
