package org.firstinspires.ftc.teamcode;

public class Gamepad {
    com.qualcomm.robotcore.hardware.Gamepad gamepad;

    BinaryComponent a, b, x, y,
            dpad_up, dpad_right, dpad_down, dpad_left,
            left_bumper, right_bumper,
            right_stick_button, left_stick_button;
    AnalogComponent left_stick_x, left_stick_y, right_stick_x, right_stick_y,
            right_trigger, left_trigger;

    public Gamepad(com.qualcomm.robotcore.hardware.Gamepad gamepad) {
        a = new BinaryComponent();
        b = new BinaryComponent();
        x = new BinaryComponent();
        y = new BinaryComponent();
        dpad_up = new BinaryComponent();
        dpad_right = new BinaryComponent();
        dpad_down = new BinaryComponent();
        dpad_left = new BinaryComponent();
        left_bumper = new BinaryComponent();
        right_bumper = new BinaryComponent();
        right_stick_button = new BinaryComponent();
        left_stick_button = new BinaryComponent();
        left_stick_x = new AnalogComponent();
        left_stick_y = new AnalogComponent();
        right_stick_x = new AnalogComponent();
        right_stick_y = new AnalogComponent();
        right_trigger = new AnalogComponent();
        left_trigger = new AnalogComponent();
    }

    public void update() {
        a.update(gamepad.a);
        b.update(gamepad.b);
        x.update(gamepad.x);
        y.update(gamepad.y);
        dpad_up.update(gamepad.dpad_up);
        dpad_right.update(gamepad.dpad_right);
        dpad_down.update(gamepad.dpad_down);
        dpad_left.update(gamepad.dpad_left);
        left_bumper.update(gamepad.left_bumper);
        right_bumper.update(gamepad.right_bumper);
        right_stick_button.update(gamepad.right_stick_button);
        left_stick_button.update(gamepad.left_stick_button);
        left_stick_x.update(gamepad.left_stick_x);
        left_stick_y.update(gamepad.left_stick_y);
        right_stick_x.update(gamepad.right_stick_x);
        right_stick_y.update(gamepad.right_stick_y);
        right_trigger.update(gamepad.right_trigger);
        left_trigger.update(gamepad.left_trigger);
    }
}