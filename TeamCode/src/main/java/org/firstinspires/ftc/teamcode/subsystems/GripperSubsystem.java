package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Constants.GripperState;

public class GripperSubsystem extends SubsystemBase {

    private final Servo gripper;

    public GripperSubsystem(HardwareMap hardwareMap, Servo gripper) {
        this.gripper = gripper;
        setPos(0);
    }

    public GripperSubsystem(HardwareMap hardwareMap, String gripper) {
        this(
                hardwareMap,
                hardwareMap.get(Servo.class, gripper)
        );
    }

    public void setPos(double percent) {
        gripper.setPosition((GripperState.OPEN.pos - GripperState.CLOSE.pos) * percent + GripperState.CLOSE.pos);

    }
    
    public double getPos() {
        return (gripper.getPosition() - GripperState.CLOSE.pos) / (GripperState.OPEN.pos - GripperState.CLOSE.pos);
    }

    public void open() {
        setPos(1.0);
    }

    public void close() {
        setPos(0.0);
    }

}
