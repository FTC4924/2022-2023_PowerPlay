package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.ANALOG_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.CONTROLLER_ELEMENT_STATE.HELD;
import static org.firstinspires.ftc.teamcode.Constants.CONTROLLER_ELEMENT_STATE.IDLE;
import static org.firstinspires.ftc.teamcode.Constants.CONTROLLER_ELEMENT_STATE.PRESSED;
import static org.firstinspires.ftc.teamcode.Constants.CONTROLLER_ELEMENT_STATE.RELEASED;


public class AnalogComponent {
    double component;

    private Constants.CONTROLLER_ELEMENT_STATE state;

    public AnalogComponent() {
        state = IDLE;
    }

    public Constants.CONTROLLER_ELEMENT_STATE getState() {
        return state;
    }



    public void update (double component) {
        boolean active = Math.abs(component) >= ANALOG_THRESHOLD;
        switch (state) {
            case IDLE:
                if (active) state = PRESSED;
                break;
            case PRESSED:
                state = active ? HELD : RELEASED;
                break;
            case HELD:
                this.component = component;
                if (!active) state = RELEASED;
                break;
            case RELEASED:
                state = !active ? IDLE : PRESSED;
                break;
        }
    }
}