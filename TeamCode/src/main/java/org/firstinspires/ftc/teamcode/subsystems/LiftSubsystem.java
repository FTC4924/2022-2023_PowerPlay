package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

import static org.firstinspires.ftc.teamcode.Constants.ARM_RAISER_MAX_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.ARM_RAISER_MIN_POSITION;

public class LiftSubsystem extends SubsystemBase {
    private static final double DEFAULT_POWER = 0.5;

    private final DcMotorEx lift;
    private final DigitalChannel liftLimit;
    private boolean liftOverride;
    private int liftOffset;
    private boolean stopLift;

    public LiftSubsystem(HardwareMap hardwareMap, DcMotorEx lift, DigitalChannel liftLimit) {
        this.lift = lift;
        this.liftLimit = liftLimit;

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift.setPower(DEFAULT_POWER);

        liftLimit.setMode(DigitalChannel.Mode.INPUT);

        new Trigger(this::limitPressed).whenActive(this::reset).whenInactive(this::stopAndReset);
    }

    public LiftSubsystem(HardwareMap hardwareMap, String lift, String liftLimitSwitch) {
        this(
                hardwareMap,
                hardwareMap.get(DcMotorEx.class, lift),
                hardwareMap.get(DigitalChannel.class, liftLimitSwitch)
        );
    }

    private boolean limitPressed() {
        return !liftLimit.getState();
    }

    private void reset() {
        liftOffset = lift.getCurrentPosition();
    }

    private void stopAndReset() {
        if (liftOverride) stop();
        reset();
    }

    public void setPos(Constants.LiftPos liftPos) {
        setPos(liftPos.pos);
    }

    public void setPos(double percent) {
        if (percent <= 0.0) zero();
        else setPos((int) Math.round((ARM_RAISER_MAX_POSITION - ARM_RAISER_MIN_POSITION) * percent) + ARM_RAISER_MIN_POSITION);
    }

    public void setPos(int pos) {
        if (pos <= 0) zero();
        else lift.setTargetPosition(pos + liftOffset);
        stopLift = false;
    }

    public void setPower(double power) {
        lift.setPower(power);
    }

    public void stop() {
        if (!stopLift) {
            lift.setTargetPosition(lift.getCurrentPosition());
            lift.setPower(DEFAULT_POWER);
        }
        stopLift = true;
    }

    public void zero() {
        lift.setTargetPosition(Integer.MIN_VALUE);
    }

    public int getPos() {
        return lift.getCurrentPosition() - liftOffset;
    }

    public int getTargetPos() {
        return lift.getTargetPosition() - liftOffset;
    }

    public boolean isBusy() {
        return lift.isBusy();
    }

    public void setOverride(boolean override) {
        liftOverride = override;
    }

}
