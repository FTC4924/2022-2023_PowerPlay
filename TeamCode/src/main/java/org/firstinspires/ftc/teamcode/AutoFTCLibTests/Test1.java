package org.firstinspires.ftc.teamcode.AutoFTCLibTests;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.ftclib.Drive;
import org.firstinspires.ftc.teamcode.NewAutoBase;

@Autonomous
public class Test1 extends NewAutoBase {
    @Override
    protected Command getCommands() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new Drive(drive, 0.25, 0, 0, 5.0),
                    new InstantCommand(gripper::close)
                ),
                new InstantCommand(gripper::open)
        );
    }
}
