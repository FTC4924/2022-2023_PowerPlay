package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.CONTROLLER_ELEMENT_STATE.HELD;
import static org.firstinspires.ftc.teamcode.Constants.CONTROLLER_ELEMENT_STATE.IDLE;
import static org.firstinspires.ftc.teamcode.Constants.CONTROLLER_ELEMENT_STATE.PRESSED;
import static org.firstinspires.ftc.teamcode.Constants.CONTROLLER_ELEMENT_STATE.RELEASED;


public class BinaryComponent {
    private Constants.CONTROLLER_ELEMENT_STATE state;

    public BinaryComponent() {
        state = IDLE;
    }

    public Constants.CONTROLLER_ELEMENT_STATE getState() {
        return state;
    }

    public void update (boolean down) {
        switch (state) {
            case IDLE:
                if (down) state = PRESSED;
                break;
            case PRESSED:
                state = down ? HELD : RELEASED;
                break;
            case HELD:
                if (!down) state = RELEASED;
                break;
            case RELEASED:
                state = !down ? IDLE : PRESSED;
                break;
        }
    }
}