package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RoadRunnerSubsystem;

public abstract class NewAutoBase extends CommandOpMode {
    protected DriveSubsystem drive;
    protected ClawSubsystem gripper;
    protected RoadRunnerSubsystem roadRunner;
    @Override
    public void initialize() {
        drive = new DriveSubsystem(
                hardwareMap,
                "leftFront",
                "rightFront",
                "leftBack",
                "rightBack"
        );

        gripper = new ClawSubsystem(hardwareMap, "gripper");

        roadRunner = new RoadRunnerSubsystem(hardwareMap, null);

        schedule(new InstantCommand().andThen(getCommands()));

        register(drive, gripper);
    }

    protected abstract Command getCommands();
}
