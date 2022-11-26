package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.ResourceManager;
import org.firstinspires.ftc.teamcode.Subsystems.RobotClaw;

import androidx.annotation.NonNull;

public class Claw extends Command {
    private RobotClaw claw;

    @Override
    public boolean start(@NonNull ResourceManager resourceManager) {
        init(resourceManager);

        claw = resourceManager.removeSubsystem(RobotClaw.class, "claw");
        if (claw == null) return false;
    }
}
