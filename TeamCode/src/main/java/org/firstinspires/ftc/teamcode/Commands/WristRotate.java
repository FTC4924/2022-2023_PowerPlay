package org.firstinspires.ftc.teamcode.Commands;

import androidx.annotation.FloatRange;

import static org.firstinspires.ftc.teamcode.Constants.CLAW_ROTATOR_COLLECTING_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.CLAW_ROTATOR_SCORING_POSITION;

public class WristRotate extends Command {

    /**
     * Rotate the wrist
     * @param percent The percent to rotate. 0 is collect, 1 is score.
     */
    public WristRotate(@FloatRange(from = 0.0, to = 1.0) double percent) {
        this.distance = CLAW_ROTATOR_COLLECTING_POSITION + percent * (CLAW_ROTATOR_SCORING_POSITION - CLAW_ROTATOR_COLLECTING_POSITION);
    }
}
