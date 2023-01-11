package org.firstinspires.ftc.teamcode.Commands.ftclib;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing.Timer;

import org.firstinspires.ftc.teamcode.Constants.WristState;
import org.firstinspires.ftc.teamcode.Constants.GripperState;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

import java.util.concurrent.TimeUnit;

public class Wrist extends CommandBase {
    private final ClawSubsystem armSubsystem;
    private final double percent;
    private final Timer timer;

    public Wrist(ClawSubsystem armSubsystem, WristState armPos, double duration) {
        this(armSubsystem, ((armPos == WristState.SCORE) ? 0 : 1), duration);
    }

    public Wrist(ClawSubsystem armSubsystem, double percent, double duration) {
        this.armSubsystem = armSubsystem;
        this.percent = percent;
        timer = new Timer((int) Math.round(duration * 1000), TimeUnit.MILLISECONDS);
    }

    @Override
    public void initialize() {
        armSubsystem.setWrist(percent);
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return timer.done();
    }
}
