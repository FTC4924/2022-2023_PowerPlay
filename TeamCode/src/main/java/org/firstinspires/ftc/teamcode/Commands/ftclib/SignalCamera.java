package org.firstinspires.ftc.teamcode.Commands.ftclib;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.visionpipelines.SignalDetectionPipeline;

public class SignalCamera extends CommandBase {
    private static Constants.SignalSide signalSide;
    private final SignalDetectionPipeline pipeline;

    public SignalCamera(SignalDetectionPipeline pipeline) {
        this.pipeline = pipeline;
    }

    @Override
    public void initialize() {
        pipeline.clearSignalSide();
    }

    @Override
    public boolean isFinished() {
        return pipeline.getLoops() >= 20;
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) signalSide = pipeline.getSignalSide();
    }

    public static Constants.SignalSide getSignalSide() {
        return signalSide;
    }
}
