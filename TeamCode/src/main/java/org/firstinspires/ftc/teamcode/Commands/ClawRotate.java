 package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Constants.*;

import org.firstinspires.ftc.teamcode.ResourceManager;
import org.firstinspires.ftc.teamcode.Subsystems.RobotClaw;

import androidx.annotation.NonNull;

public class ClawRotate extends Command {
    private RobotClaw claw;
    private final ClawRotatorPos clawPos;
    private final double duration;
    private double startTime;

    public ClawRotate(ClawRotatorPos clawPos, double duration) {
        this.clawPos = clawPos;
        this.duration = duration;
    }

    @Override
    public boolean start(@NonNull ResourceManager resourceManager) {
        init(resourceManager);

        claw = resourceManager.removeSubsystem(RobotClaw.class, "robotClaw");
        if (claw == null) return false;

        claw.setRotatorPos(clawPos.pos);

        startTime = System.nanoTime() / 1e9;

        return true;
    }

    @Override
    public boolean update() {
        return Math.abs(startTime - System.nanoTime() / 1e9) < duration;
    }

    @Override
    public void stop(@NonNull ResourceManager resourceManager) {
        resourceManager.addSubsystems(claw);
    }
}