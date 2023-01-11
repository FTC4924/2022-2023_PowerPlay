package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Constants.*;

public class ArmSubsystem extends SubsystemBase {

    private final DcMotorEx lift;
    private final DcMotorEx arm;
    private final DigitalChannel liftLimit;
    private final DigitalChannel armLimit;
    private final Trigger liftPressed;
    private final Trigger armPressed;
    private int liftOffset;
    private int armOffset;

    private int armZeroMesurement;
    private int armZeroDirection;

    public ArmSubsystem(HardwareMap hardwareMap, DcMotorEx lift, DcMotorEx arm, DigitalChannel liftLimit, DigitalChannel armLimit) {
        this.lift = lift;
        this.arm = arm;
        this.liftLimit = liftLimit;

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);

        liftLimit.setMode(DigitalChannel.Mode.INPUT);
        //this.armLimit.setMode(DigitalChannel.Mode.INPUT);
        this.armLimit = null;

        liftPressed = new Trigger(this::liftPressed).whenActive(this::resetLift).whenInactive(this::stopAndResetLift);
        armPressed = new Trigger(null).whenActive(this::resetArm).whenInactive(this::resetArm);
    }

    public ArmSubsystem(HardwareMap hardwareMap, String lift, String arm, String liftLimitSwitch, String armLimitSwitch) {
        this(
                hardwareMap,
                hardwareMap.get(DcMotorEx.class, lift),
                hardwareMap.get(DcMotorEx.class, arm),
                hardwareMap.get(DigitalChannel.class, liftLimitSwitch),
                null // TODO: 1/10/2023 Change to getting the switch from the hardware map once the switch is installed.
        );
    }

    private boolean liftPressed() {
        return !liftLimit.getState();
    }

    private boolean armPressed() {
        return !armLimit.getState();
    }

    private void resetLift() {
        liftOffset = lift.getCurrentPosition();
    }

    private void stopAndResetLift() {
        lift.setTargetPosition(lift.getCurrentPosition());
        resetLift();
    }

    private void resetArm() {
        if (armZeroDirection == 0) {  // Store the first value for zeroing the arm.
            armZeroMesurement = arm.getCurrentPosition();
            armZeroDirection = Integer.signum(arm.getTargetPosition() - arm.getCurrentPosition());
            return;
        }

        if (Integer.signum(arm.getTargetPosition() - arm.getCurrentPosition()) != armZeroDirection) {  // Detect if we have reversed and come back out the other direction.
            armZeroDirection = 0;
            return;
        }

        armOffset = (arm.getCurrentPosition() + armZeroMesurement) / 2;
        armZeroDirection = 0;
    }

    public void setLift(LiftPos liftPos) {
        setLift(liftPos.pos);
    }

    public void setLift(double percent) {
        if (percent <= 0.0) zeroLift();
        else setLift((int) Math.round((ARM_RAISER_MAX_POSITION - ARM_RAISER_MIN_POSITION) * percent) + ARM_RAISER_MIN_POSITION);
    }

    public void setLift(int pos) {
        if (pos <= 0) zeroLift();
        else lift.setTargetPosition(pos + liftOffset);
    }

    public void setLiftPower(double power) {
        lift.setPower(power);
    }

    public void zeroLift() {
        lift.setTargetPosition(Integer.MIN_VALUE);
    }

    public void setArm(ArmPos armPos) {
        setArm(armPos.pos);
    }

    public void setArm(double percent) {
        setArm((int) Math.round((ArmPos.ARM_SCORING.pos - ArmPos.ARM_COLLECTING.pos) * percent) + ArmPos.ARM_COLLECTING.pos);
    }

    public void setArm(int pos) {
        arm.setTargetPosition(pos + armOffset);
    }

    public void setArmPower(double power) {
        arm.setPower(power);
    }

    public int getLiftPos() {
        return lift.getCurrentPosition();
    }

    public int getArmPos() {
        return arm.getCurrentPosition();
    }

    public boolean getLiftBusy() {
        return lift.isBusy();
    }

    public boolean getArmBusy() {
        return arm.isBusy();
    }
}
