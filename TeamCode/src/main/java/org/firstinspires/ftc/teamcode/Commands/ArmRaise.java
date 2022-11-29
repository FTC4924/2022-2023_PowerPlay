package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Constants.ENCODER_POSITION_TOLERANCE;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ResourceManager;
import org.firstinspires.ftc.teamcode.Subsystems.RobotArm;

public class ArmRaise extends Command {
    private RobotArm robotArm;
    private final int raiserPos;
    private final double raiserPower;

    public ArmRaise(int raiserPos, double raiserPower) {
        this.raiserPos = raiserPos;
        this.raiserPower = raiserPower;
    }

    @Override
    public boolean start(@NonNull ResourceManager resourceManager) {
        init(resourceManager);

        robotArm = resourceManager.removeSubsystem(RobotArm.class, "robotArm");
        if (robotArm == null) return false;

        robotArm.setRaiserPos(raiserPos);
        robotArm.setRaiserPower(raiserPower);

        return true;
    }

    @Override
    public boolean update() {
        return Math.abs(raiserPos - robotArm.getRaiserPos()) <= ENCODER_POSITION_TOLERANCE;
    }

    @Override
    public void stop(@NonNull ResourceManager resourceManager) {
        resourceManager.addSubsystems(robotArm);
    }
}
