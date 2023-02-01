package org.firstinspires.ftc.teamcode.Commands.ftclib.defaultcommands;

import static org.firstinspires.ftc.teamcode.Constants.ANALOG_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.ArmPos;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultArm extends CommandBase {
    private final ArmSubsystem arm;
    private final DoubleSupplier y;

    public DefaultArm(ArmSubsystem arm, DoubleSupplier y) {
        this.arm = arm;
        this.y = y;

        addRequirements(arm);
    }

    @Override
    public void execute() {
        if (Math.abs(y.getAsDouble()) < ANALOG_THRESHOLD) {
            arm.stop();
            return;
        }

        if (y.getAsDouble() > 0) arm.setPos(ArmPos.ARM_SCORING);
        else arm.setPos(ArmPos.ARM_COLLECTING);

        arm.setPower(y.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}