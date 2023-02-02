package org.firstinspires.ftc.teamcode.Commands.ftclib;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants.LiftPos;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class Lift extends CommandBase {
    private final LiftSubsystem lift;
    private final double percent;
    private final double power;

    public Lift(LiftSubsystem lift, LiftPos armPos, double power) {
        this(lift, ((armPos == LiftPos.LIFT_MIN) ? 0 : 1), power);
    }

    public Lift(LiftSubsystem lift, double percent, double power) {
        this.lift = lift;
        this.percent = percent;
        this.power = power;
    }

    @Override
    public void initialize() {
        lift.setPower(power);
        lift.setPos(percent);
    }

    @Override
    public boolean isFinished() {
        return !lift.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) lift.setPos(lift.getPos());
    }
}
