package org.firstinspires.ftc.teamcode.Commands.ftclib;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants.ArmPos;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class Arm extends CommandBase {
    private final ArmSubsystem arm;
    private final double percent;
    private final double power;

    public Arm(ArmSubsystem arm, ArmPos armPos, double power) {
        this(arm, ((armPos == ArmPos.ARM_COLLECTING) ? 0 : 1), power);
    }

    public Arm(ArmSubsystem arm, double percent, double power) {
        this.arm = arm;
        this.percent = percent;
        this.power = power;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setPower(power);
        arm.setPos(percent);
    }

    @Override
    public boolean isFinished() {
        return !arm.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) arm.setPos(arm.getPos());
    }
}
