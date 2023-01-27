package org.firstinspires.ftc.teamcode.triggers;

import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.Constants;

import java.util.function.Supplier;

public class TeleopStateTrigger extends Trigger {
    private final Supplier<Constants.TELEOP_STATE> value;
    private final Constants.TELEOP_STATE target;

    public TeleopStateTrigger(Supplier<Constants.TELEOP_STATE> value, Constants.TELEOP_STATE target) {
        this.value = value;
        this.target = target;
    }

    @Override
    public boolean get() {
        return value.get() == target;
    }
}
