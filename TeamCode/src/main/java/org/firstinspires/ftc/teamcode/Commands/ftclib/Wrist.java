package org.firstinspires.ftc.teamcode.Commands.ftclib;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing.Timer;

import org.firstinspires.ftc.teamcode.Constants.WristState;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

import java.util.concurrent.TimeUnit;

public class Wrist extends CommandBase {
    private final WristSubsystem wristSubsystem;
    private final double percent;
    private final Timer timer;

    public Wrist(WristSubsystem wristSubsystem, WristState armPos, double duration) {
        this(wristSubsystem, ((armPos == WristState.SCORE) ? 0 : 1), duration);
    }

    public Wrist(WristSubsystem wristSubsystem, double percent, double duration) {
        this.wristSubsystem = wristSubsystem;
        this.percent = percent;
        timer = new Timer((int) Math.round(duration * 1000), TimeUnit.MILLISECONDS);
    }

    @Override
    public void initialize() {
        wristSubsystem.setPos(percent);
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return timer.done();
    }
}
