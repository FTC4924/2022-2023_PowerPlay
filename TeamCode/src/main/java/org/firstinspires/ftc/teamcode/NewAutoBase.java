package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public abstract class NewAutoBase extends CommandOpMode {
    protected DriveSubsystem drive;
    protected ClawSubsystem gripper;
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

        schedule(new InstantCommand().andThen(getCommands()));

        register(drive, gripper);
    }

    protected abstract Command getCommands();
}
