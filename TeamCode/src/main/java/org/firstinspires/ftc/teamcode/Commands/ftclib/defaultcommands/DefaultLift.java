package org.firstinspires.ftc.teamcode.Commands.ftclib.defaultcommands;

import static org.firstinspires.ftc.teamcode.Constants.ANALOG_THRESHOLD;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DefaultLift extends CommandBase {
    private final LiftSubsystem lift;
    private final DoubleSupplier y;
    private final BooleanSupplier rStickButton;
    private final Telemetry telemetry;

    public DefaultLift(LiftSubsystem lift, DoubleSupplier y, BooleanSupplier rStickButton, Telemetry telemetry) {
        this.lift = lift;
        this.y = y;
        this.rStickButton = rStickButton;
        this.telemetry = telemetry;

        addRequirements(lift);
    }

    @Override
    public void execute() {
        telemetry.addData("DefaultLift", y.getAsDouble());
        if (Math.abs(y.getAsDouble()) <= ANALOG_THRESHOLD) {
            lift.stop();
            return;
        }

        /*if (rStickButton.getAsBoolean()) {
            liftO();
        } else {
            liftM();
        }*/
        liftM();
    }

    @Override
    public void end(boolean interrupted) {
        lift.stop();
    }

    private void liftM() {
        lift.setOverride(true);

        if (y.getAsDouble() > 0) lift.setPos(Constants.LiftPos.LIFT_UP);
        else lift.setPos(Constants.LiftPos.LIFT_DOWN);

        lift.setPower(y.getAsDouble());
    }

    private void liftO() {
        lift.setOverride(false);
        lift.setPos(lift.getPos() + (int) (y.getAsDouble() * 100));
        lift.setPower(y.getAsDouble());
    }
}
