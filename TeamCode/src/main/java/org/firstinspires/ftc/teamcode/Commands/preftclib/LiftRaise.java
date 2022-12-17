package org.firstinspires.ftc.teamcode.Commands.preftclib;

import androidx.annotation.FloatRange;

import static org.firstinspires.ftc.teamcode.Constants.ARM_RAISER_COLLECTING_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.ARM_RAISER_SCORING_POSITION;

public class LiftRaise extends Command {

    /**
     * Changes the lifter height.
     * @param percent The percent height to move to. 0 is the min height, 1 is the max height.
     */
    public LiftRaise(@FloatRange(from = 0.0, to = 1.0) double percent) {
        this.position = (int)(ARM_RAISER_COLLECTING_POSITION + percent * (ARM_RAISER_SCORING_POSITION - ARM_RAISER_COLLECTING_POSITION));
    }
}
