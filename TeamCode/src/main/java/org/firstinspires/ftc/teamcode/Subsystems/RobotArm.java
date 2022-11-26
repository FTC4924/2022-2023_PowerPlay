package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.ResourceManager;

import androidx.annotation.NonNull;

public class RobotArm extends Subsystem {
    private final DcMotorEx armRaiser;
    private final DcMotorEx armRotator;

    public RobotArm(@NonNull ResourceManager resourceManager, String name) {
        super(resourceManager, name);

        armRaiser = resourceManager.removeDevice(DcMotorEx.class, "armRaiser");
        armRotator = resourceManager.removeDevice(DcMotorEx.class, "armRotator");

        armRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotator.setTargetPosition(0);
        armRotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRaiser.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRaiser.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRaiser.setTargetPosition(0);
        armRaiser.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int getRaiserPos() {
        return armRaiser.getCurrentPosition();
    }

    public int getRotatorPos() {
        return armRotator.getCurrentPosition();
    }

    public double getRaiserPower() {
        return armRaiser.getPower();
    }

    public double getRotatorPower() {
        return armRotator.getPower();
    }

    public void setRaiserPos(int pos) {
        armRaiser.setTargetPosition(pos);
    }

    public void setRotatorPos(int pos) {
        armRotator.setTargetPosition(pos);
    }

    public void setRaiserPower(double power) {
        armRaiser.setPower(power);
    }

    public void setRotatorPower(double power) {
        armRotator.setPower(power);
    }
}
