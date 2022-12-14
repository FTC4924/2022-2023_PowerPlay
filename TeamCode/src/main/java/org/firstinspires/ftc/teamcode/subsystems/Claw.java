package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw extends SubsystemBase {

    private final Servo gripper;

    public Claw(final HardwareMap hMap, final String name) {
        gripper = hMap.get(Servo.class, name);
    }

    /**
     * Grabs a stone.
     */
    public void grab() {
        gripper.setPosition(0.76);
    }

    /**
     * Releases a stone.
     */
    public void release() {
        gripper.setPosition(0);
    }

}
