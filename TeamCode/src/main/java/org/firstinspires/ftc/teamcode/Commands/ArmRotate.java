package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Constants.ENCODER_POSITION_TOLERANCE;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ResourceManager;
import org.firstinspires.ftc.teamcode.Subsystems.RobotArm;

public class ArmRotate extends Command {
    private RobotArm robotArm;
    private final int rotatorPos;
    private final double rotatorPower;

    public ArmRotate(int rotatorPos, double rotatorPower) {
        this.rotatorPos = rotatorPos;
        this.rotatorPower = rotatorPower;
    }

    @Override
    public boolean start(@NonNull ResourceManager resourceManager) {
        init(resourceManager);

        robotArm = resourceManager.removeSubsystem(RobotArm.class, "robotArm");
        if (robotArm == null) return false;

        robotArm.setRotatorPos(rotatorPos);
        robotArm.setRotatorPower(rotatorPower);

        return true;
    }

    @Override
    public boolean update() {
        return Math.abs(rotatorPos - robotArm.getRotatorPos()) <= ENCODER_POSITION_TOLERANCE;
    }

    @Override
    public void stop(@NonNull ResourceManager resourceManager) {
        resourceManager.addSubsystems(robotArm);
    }
}
