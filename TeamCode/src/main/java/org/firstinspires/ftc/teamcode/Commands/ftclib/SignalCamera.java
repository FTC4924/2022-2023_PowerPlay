package org.firstinspires.ftc.teamcode.Commands.ftclib;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.visionpipelines.SignalDetectionPipeline;

public class SignalCamera extends CommandBase {
    public static Constants.SignalSide signalSide;
    private final SignalDetectionPipeline pipeline;

    public SignalCamera() {
        pipeline = new SignalDetectionPipeline();
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand()
                )
        );
    }
}
