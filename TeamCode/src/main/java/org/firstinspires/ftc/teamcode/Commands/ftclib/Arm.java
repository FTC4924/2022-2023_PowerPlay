package org.firstinspires.ftc.teamcode.Commands.ftclib;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants.ArmPos;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class Arm extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final double percent;
    private final double power;

    public Arm(ArmSubsystem armSubsystem, ArmPos armPos, double power) {
        this(armSubsystem, ((armPos == ArmPos.ARM_COLLECTING) ? 0 : 1), power);
    }

    public Arm(ArmSubsystem armSubsystem, double percent, double power) {
        this.armSubsystem = armSubsystem;
        this.percent = percent;
        this.power = power;
    }

    @Override
    public void initialize() {
        armSubsystem.setArmPower(power);
        armSubsystem.setArm(percent);
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.getArmBusy();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) armSubsystem.setArm(armSubsystem.getArmPos());
    }
}
