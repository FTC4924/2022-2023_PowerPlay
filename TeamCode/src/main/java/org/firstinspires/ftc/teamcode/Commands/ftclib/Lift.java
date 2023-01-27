package org.firstinspires.ftc.teamcode.Commands.ftclib;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants.LiftPos;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class Lift extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final double percent;
    private final double power;

    public Lift(ArmSubsystem armSubsystem, LiftPos armPos, double power) {
        this(armSubsystem, ((armPos == LiftPos.LIFT_DOWN) ? 0 : 1), power);
    }

    public Lift(ArmSubsystem armSubsystem, double percent, double power) {
        this.armSubsystem = armSubsystem;
        this.percent = percent;
        this.power = power;
    }

    @Override
    public void initialize() {
        armSubsystem.setLiftPower(power);
        armSubsystem.setLift(percent);
    }

    @Override
    public boolean isFinished() {
        return !armSubsystem.getLiftBusy();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) armSubsystem.setLift(armSubsystem.getLiftPos());
    }
}
