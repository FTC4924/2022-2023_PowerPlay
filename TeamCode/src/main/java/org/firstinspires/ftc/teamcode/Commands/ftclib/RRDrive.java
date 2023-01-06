package org.firstinspires.ftc.teamcode.Commands.ftclib;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.RoadRunnerSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class RRDrive extends CommandBase {

    private final TrajectorySequence trajectory;
    private final RoadRunnerSubsystem roadRunnerSubsystem;

    public RRDrive(RoadRunnerSubsystem roadRunnerSubsystem, TrajectorySequence trajectory) {
        this.trajectory = trajectory;
        this.roadRunnerSubsystem = roadRunnerSubsystem;
        addRequirements(roadRunnerSubsystem);

    }

    @Override
    public void initialize() {
        roadRunnerSubsystem.followTrajectorySequence(trajectory);
    }

    @Override
    public boolean isFinished() {
        return !roadRunnerSubsystem.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) roadRunnerSubsystem.breakFollowing();
    }
}
