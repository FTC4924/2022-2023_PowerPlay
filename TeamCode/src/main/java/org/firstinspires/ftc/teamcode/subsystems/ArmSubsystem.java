package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.button.Button;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmSubsystem extends SubsystemBase {

    private final DcMotorEx lift;
    private final DcMotorEx arm;
    private final DigitalChannel liftLimit;
    private final DigitalChannel armLimit;
    private final Button liftPressed;
    private final Button armPressed;

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

        liftPressed = new Button(this::liftPressed) {};
        armPressed = null;
    }

    public ArmSubsystem(HardwareMap hardwareMap, String lift, String arm, String liftLimitSwitch, String armLimitSwitch) {
        this(
                hardwareMap,
                hardwareMap.get(DcMotorEx.class, lift),
                hardwareMap.get(DcMotorEx.class, arm),
                hardwareMap.get(DigitalChannel.class, liftLimitSwitch),
                null
        );
    }

    private boolean liftPressed() {
        return !liftLimit.getState();
    }

    private boolean armPressed() {
        return !armLimit.getState();
    }

    public void setLift(int pos) {

    }
}
