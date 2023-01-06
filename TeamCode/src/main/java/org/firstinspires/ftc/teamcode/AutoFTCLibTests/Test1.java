package org.firstinspires.ftc.teamcode.AutoFTCLibTests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.ftclib.RRDrive;
import org.firstinspires.ftc.teamcode.NewAutoBase;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class Test1 extends NewAutoBase {
    @Override
    protected Command getCommands() {
        TrajectorySequence traj1 = roadRunner.trajectorySequenceBuilder(new Pose2d())
                .forward(10)
                .build();
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new RRDrive(roadRunner, traj1),
                        new InstantCommand(gripper::close)
                ),
                new InstantCommand(gripper::open)
        );
    }
}
