package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Constants.WristState;

public class WristSubsystem extends SubsystemBase {

    private final Servo wrist;

    public WristSubsystem(HardwareMap hardwareMap, Servo wrist) {
        this.wrist = wrist;
    }

    public WristSubsystem(HardwareMap hardwareMap, String wrist) {
        this(
                hardwareMap,
                hardwareMap.get(Servo.class, wrist)
        );
    }

    public void setPos(double percent) {
        wrist.setPosition((WristState.COLLECT.pos - WristState.SCORE.pos) * percent + WristState.SCORE.pos);
    }


    public double getPos() {
        return (wrist.getPosition() - WristState.SCORE.pos) / (WristState.COLLECT.pos - WristState.SCORE.pos);
    }



    public void wristCollect() {
        setPos(1.0);
    }

    public void wristScore() {
        setPos(0.0);
    }
}
