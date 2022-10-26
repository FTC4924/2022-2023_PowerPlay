package org.firstinspires.ftc.teamcode;



public class Button {

    boolean pressed;
    boolean held;
    boolean released;



    public boolean getPressed() {
        return pressed;
    }
    public boolean getHeld() {
        return held;
    }
    public boolean getReleased() {
        return released;
    }
    public void update (boolean held) {
        // Logic for pressed
        if (!held && this.held) {
            pressed = true;
        } else if (pressed) {
            pressed = false;
        }
        // Logic for released
        if (held && !this.held) {
            released = true;
        } else if (released) {
            released = false;
        }
        // Logic for held
        this.held = held;
    }

}
