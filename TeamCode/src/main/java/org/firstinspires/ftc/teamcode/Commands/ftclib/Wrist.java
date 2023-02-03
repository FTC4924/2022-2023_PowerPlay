package org.firstinspires.ftc.teamcode.Commands.ftclib;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing.Timer;

import org.firstinspires.ftc.teamcode.Constants.WristState;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

import java.util.concurrent.TimeUnit;

public class Wrist extends CommandBase {
    private final WristSubsystem wrist;
    private final double percent;
    private final Timer timer;

    public Wrist(WristSubsystem wrist, WristState wristPos, double duration) {
        this(wrist, ((wristPos == WristState.SCORE) ? 0 : 1), duration);
    }

    public Wrist(WristSubsystem wrist, double percent, double duration) {
        this.wrist = wrist;
        this.percent = percent;
        timer = new Timer((int) Math.round(duration * 1000), TimeUnit.MILLISECONDS);

        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        wrist.setPos(percent);
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return timer.done();
    }
}
