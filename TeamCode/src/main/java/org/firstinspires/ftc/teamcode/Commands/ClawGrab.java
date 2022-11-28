package org.firstinspires.ftc.teamcode.Commands;

import androidx.annotation.NonNull;

import static org.firstinspires.ftc.teamcode.Constants.*;
import org.firstinspires.ftc.teamcode.ResourceManager;
import org.firstinspires.ftc.teamcode.Subsystems.RobotClaw;

public class ClawGrab extends Command {
    private RobotClaw claw;
    private final ClawGrabberPos grabberPos;
    private final double duration;
    private double startTime;

    public ClawGrab(ClawGrabberPos grabberPos, double duration) {
        this.grabberPos = grabberPos;
        this.duration = duration;
    }

    @Override
    public boolean start(@NonNull ResourceManager resourceManager) {
        init(resourceManager);

        claw = resourceManager.removeSubsystem(RobotClaw.class, "robotClaw");
        if (claw == null) return false;

        claw.setGripperPos(grabberPos.pos);

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
