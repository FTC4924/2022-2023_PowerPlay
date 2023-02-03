package org.firstinspires.ftc.teamcode.Commands.ftclib;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants.LiftPos;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class Lift extends CommandBase {
    private final LiftSubsystem lift;
    private final int pos;
    private final double power;

    public Lift(LiftSubsystem lift, LiftPos liftPos, double power) {
        this(lift, liftPos.pos, power);
    }

    public Lift(LiftSubsystem lift, double percent, double power) {
        this(lift, (int) ((LiftPos.LIFT_MAX.pos - LiftPos.LIFT_MIN.pos) * percent + LiftPos.LIFT_MIN.pos), power);
    }

    public Lift(LiftSubsystem lift, int pos, double power) {
        this.lift = lift;
        this.pos = pos;
        this.power = power;

        addRequirements(lift);
    }

    @Override
    public void initialize() {
        lift.setPower(power);
        lift.setPos(pos);
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
