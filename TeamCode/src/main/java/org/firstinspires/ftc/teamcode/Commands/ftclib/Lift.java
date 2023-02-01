package org.firstinspires.ftc.teamcode.Commands.ftclib;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants.LiftPos;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class Lift extends CommandBase {
    private final LiftSubsystem liftSubsystem;
    private final double percent;
    private final double power;

    public Lift(LiftSubsystem liftSubsystem, LiftPos armPos, double power) {
        this(liftSubsystem, ((armPos == LiftPos.LIFT_DOWN) ? 0 : 1), power);
    }

    public Lift(LiftSubsystem liftSubsystem, double percent, double power) {
        this.liftSubsystem = liftSubsystem;
        this.percent = percent;
        this.power = power;
    }

    @Override
    public void initialize() {
        liftSubsystem.setPower(power);
        liftSubsystem.setPos(percent);
    }

    @Override
    public boolean isFinished() {
        return !liftSubsystem.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) liftSubsystem.setPos(liftSubsystem.getPos());
    }
}
