package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Constants.CLAW_GRABBER_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.CLAW_GRABBER_OPEN_POSITION;

public class ClawSubsystem extends SubsystemBase {

    private final Servo gripper;

    public ClawSubsystem(final HardwareMap hMap, final String name) {
        gripper = hMap.get(Servo.class, name);
    }

    /**
     * Grabs a stone.
     */
    public void close() {
        gripper.setPosition(CLAW_GRABBER_CLOSE_POSITION);
    }

    /**
     * Releases a stone.
     */
    public void open() {
        gripper.setPosition(CLAW_GRABBER_OPEN_POSITION);
    }

}
