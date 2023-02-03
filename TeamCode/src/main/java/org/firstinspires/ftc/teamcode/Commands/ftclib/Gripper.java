package org.firstinspires.ftc.teamcode.Commands.ftclib;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing.Timer;

import org.firstinspires.ftc.teamcode.Constants.GripperState;
import org.firstinspires.ftc.teamcode.subsystems.GripperSubsystem;

import java.util.concurrent.TimeUnit;

public class Gripper extends CommandBase {
    private final GripperSubsystem gripper;
    private final double percent;
    private final Timer timer;

    public Gripper(GripperSubsystem gripper, GripperState armPos, double duration) {
        this(gripper, ((armPos == GripperState.CLOSE) ? 0 : 1), duration);
    }

    public Gripper(GripperSubsystem gripper, double percent, double duration) {
        this.gripper = gripper;
        this.percent = percent;
        timer = new Timer((int) Math.round(duration * 1000), TimeUnit.MILLISECONDS);
        addRequirements(gripper);
    }

    @Override
    public void initialize() {
        gripper.setPos(percent);
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return timer.done();
    }
}
