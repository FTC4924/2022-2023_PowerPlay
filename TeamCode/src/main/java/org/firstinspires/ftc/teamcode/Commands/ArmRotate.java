package org.firstinspires.ftc.teamcode.Commands;

import androidx.annotation.FloatRange;

import static org.firstinspires.ftc.teamcode.Constants.ARM_ROTATOR_COLLECTING_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.ARM_ROTATOR_SCORING_POSITION;

public class ArmRotate extends Command {

    /**
     * Rotates the arm.
     * @param percent The percent swing. 0 is the collect pos, 1 is the scoring pos.
     */
    public ArmRotate(@FloatRange(from = 0.0, to = 1.0) double percent) {
        this.position = (int)(ARM_ROTATOR_COLLECTING_POSITION + percent * (ARM_ROTATOR_SCORING_POSITION - ARM_ROTATOR_COLLECTING_POSITION));
    }

    public ArmRotate(int pos) {
        this.position = pos;
    }
}
