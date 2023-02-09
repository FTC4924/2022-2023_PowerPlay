package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Constants.ArmPos;

public class ArmSubsystem extends SubsystemBase {
    private static final int ARM_RESET_OFFSET = -624;
    private static final double DEFAULT_POWER = 0.5;
    
    private final DcMotorEx arm;
    private final DigitalChannel armLimit;
    private int armOffset;

    private int armZeroMesurement;
    private int armZeroDirection;
    private boolean ignoreLimitTemporary;

    private boolean stopArm;

    public ArmSubsystem(HardwareMap hardwareMap, DcMotorEx arm, DigitalChannel armLimit) {
        this.arm = arm;

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm.setPower(DEFAULT_POWER);

        //armOffset = ARM_RESET_OFFSET;

        this.armLimit = armLimit;
        
        armLimit.setMode(DigitalChannel.Mode.INPUT);

        ignoreLimitTemporary = pressed();

        //new Trigger(this::pressed).whenActive(this::reset)/*.whenInactive(this::reset)*/;
    }

    public ArmSubsystem(HardwareMap hardwareMap, String arm, String armLimitSwitch) {
        this(
                hardwareMap,
                hardwareMap.get(DcMotorEx.class, arm),
                hardwareMap.get(DigitalChannel.class, armLimitSwitch)
        );
    }

    public boolean pressed() { // TODO: 1/13/2023 Change for the addition of the arm limit switch
        return !armLimit.getState();
    }

    private void reset() {
        armOffset = -ARM_RESET_OFFSET + arm.getCurrentPosition() + armZeroMesurement;
        /*if (ignoreLimitTemporary && !pressed()) {  // If the limit switch was triggered at the beginning of the program, ignore it until it is unpressed.
            ignoreLimitTemporary = false;
            return;
        }

        if (armZeroDirection == 0) {  // Store the first value for zeroing the arm.
            armZeroMesurement = arm.getCurrentPosition();
            armZeroDirection = Integer.signum(arm.getTargetPosition() - arm.getCurrentPosition());
            return;
        }

        if (Integer.signum(arm.getTargetPosition() - arm.getCurrentPosition()) == armZeroDirection) {  // Detect if we have not reversed and come back out the other direction.
            armOffset = -ARM_RESET_OFFSET + (arm.getCurrentPosition() + armZeroMesurement) / 2;
        }

        armZeroDirection = 0;*/
    }

    public void setPos(ArmPos armPos) {
        setPos(armPos.pos);
    }

    public void setPos(double percent) {
        setPos((int) Math.round((ArmPos.ARM_SCORING.pos - ArmPos.ARM_COLLECTING.pos) * percent) + ArmPos.ARM_COLLECTING.pos);
    }

    public void setPos(int pos) {
        arm.setTargetPosition(pos + armOffset);
        stopArm = false;
    }

    public void stop() {
        if (!stopArm) {
            arm.setTargetPosition(arm.getCurrentPosition());
            arm.setPower(DEFAULT_POWER);
        }
        stopArm = true;
    }

    public void setPower(double power) {
        arm.setPower(power);
    }

    public int getPos() {
        return arm.getCurrentPosition() - armOffset;
    }

    public int getOffset() {
        return armOffset;
    }

    public boolean isBusy() {
        return arm.isBusy();
    }
}
