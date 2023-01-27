package org.firstinspires.ftc.teamcode.triggers;

import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.Constants;

import java.util.function.DoubleSupplier;

public class JoystickTrigger extends Trigger {
    public final AxisTrigger x;
    public final AxisTrigger y;

    public JoystickTrigger(DoubleSupplier x, DoubleSupplier y) {
        this.x = new AxisTrigger(x, Constants.ANALOG_THRESHOLD);
        this.y = new AxisTrigger(y, Constants.ANALOG_THRESHOLD);
    }
    
    @Override
    public boolean get() {
        return x.get() && y.get();
    }
}
