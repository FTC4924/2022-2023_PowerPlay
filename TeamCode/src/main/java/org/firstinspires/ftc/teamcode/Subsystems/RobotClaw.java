package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.ResourceManager;

import androidx.annotation.NonNull;

public class RobotClaw extends Subsystem {
    private final Servo clawGripper;
    private final Servo clawRotator;

    public RobotClaw(@NonNull ResourceManager resourceManager, String name) {
        super(resourceManager, name);

        clawGripper = resourceManager.removeDevice(Servo.class, "clawGrabber");
        clawRotator = resourceManager.removeDevice(Servo.class, "clawRotator");

        clawGripper.setPosition(Constants.CLAW_GRABBER_CLOSE_POSITION);
        clawRotator.setPosition(Constants.CLAW_ROTATOR_COLLECTING_POSITION);
    }

    public double getGripperPos() {
        return clawGripper.getPosition();
    }

    public double getRotatorPos() {
        return clawRotator.getPosition();
    }

    public void setGripperPos(double pos) {
        clawGripper.setPosition(pos);
    }

    public void setRotatorPos(double pos) {
        clawRotator.setPosition(pos);
    }
}
