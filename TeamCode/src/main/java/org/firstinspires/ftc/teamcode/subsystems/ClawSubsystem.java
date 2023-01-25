package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Constants.GripperState;
import static org.firstinspires.ftc.teamcode.Constants.WristState;

public class ClawSubsystem extends SubsystemBase {

    private final Servo gripper;
    private final Servo wrist;

    public ClawSubsystem(HardwareMap hardwareMap, Servo gripper, Servo wrist) {
        this.gripper = gripper;
        this.wrist = wrist;
    }

    public ClawSubsystem(HardwareMap hardwareMap, String gripper, String wrist) {
        this(
                hardwareMap,
                hardwareMap.get(Servo.class, gripper),
                hardwareMap.get(Servo.class, wrist)
        );
    }

    public void setGripper(double percent) {
        gripper.setPosition((GripperState.OPEN.pos - GripperState.CLOSE.pos) * percent + GripperState.CLOSE.pos);

    }

    public void setWrist(double percent) {
        wrist.setPosition((WristState.COLLECT.pos - WristState.SCORE.pos) * percent + WristState.SCORE.pos);
    }

    public double getGripperPos() {
        return (gripper.getPosition() - GripperState.CLOSE.pos) / (GripperState.OPEN.pos - GripperState.CLOSE.pos);
    }

    public double getWristPos() {
        return (wrist.getPosition() - WristState.SCORE.pos) / (WristState.COLLECT.pos - WristState.SCORE.pos);
    }

    public void gripperOpen() {
        setGripper(1.0);
    }

    public void gripperClose() {
        setGripper(0.0);
    }

    public void wristCollect() {
        setWrist(1.0);
    }

    public void wristScore() {
        setWrist(0.0);
    }
}
