package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Constants.ArmPos;

public class ArmSubsystem extends SubsystemBase {
    
    private final DcMotorEx arm;
    private final DigitalChannel armLimit;
    private final Trigger armPressed;
    private int armOffset;

    private int armZeroMesurement;
    private int armZeroDirection;

    public ArmSubsystem(HardwareMap hardwareMap, DcMotorEx arm, DigitalChannel armLimit) {
        this.arm = arm;

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        //this.armLimit.setMode(DigitalChannel.Mode.INPUT);
        this.armLimit = null; // TODO: 1/13/2023 Change for the addition of the arm limit switch

        armPressed = new Trigger(this::pressed).whenActive(this::reset).whenInactive(this::reset);
    }

    public ArmSubsystem(HardwareMap hardwareMap, String arm, String armLimitSwitch) {
        this(
                hardwareMap,
                hardwareMap.get(DcMotorEx.class, arm),
                null // TODO: 1/13/2023 Change for the addition of the arm limit switch
        );
    }

    private boolean pressed() { // TODO: 1/13/2023 Change for the addition of the arm limit switch
        return false;//!armLimit.getState();
    }

    private void reset() {
        if (armZeroDirection == 0) {  // Store the first value for zeroing the arm.
            armZeroMesurement = arm.getCurrentPosition();
            armZeroDirection = Integer.signum(arm.getTargetPosition() - arm.getCurrentPosition());
            return;
        }

        if (Integer.signum(arm.getTargetPosition() - arm.getCurrentPosition()) != armZeroDirection) {  // Detect if we have not reversed and come back out the other direction.
            armZeroDirection = 0;
            return;
        }

        armOffset = (arm.getCurrentPosition() + armZeroMesurement) / 2;
        armZeroDirection = 0;
    }

    public void setPos(ArmPos armPos) {
        setPos(armPos.pos);
    }

    public void setPos(double percent) {
        setPos((int) Math.round((ArmPos.ARM_SCORING.pos - ArmPos.ARM_COLLECTING.pos) * percent) + ArmPos.ARM_COLLECTING.pos);
    }

    public void setPos(int pos) {
        arm.setTargetPosition(pos + armOffset);
    }

    public void stop() {
        arm.setTargetPosition(arm.getCurrentPosition());
    }

    public void setPower(double power) {
        arm.setPower(power);
    }

    public int getPos() {
        return arm.getCurrentPosition();
    }

    public boolean isBusy() {
        return arm.isBusy();
    }
}
