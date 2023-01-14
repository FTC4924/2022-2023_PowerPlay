package org.firstinspires.ftc.teamcode.AutoFTCLibTests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.ftclib.RRDrive;
import org.firstinspires.ftc.teamcode.NewAutoBase;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class Test1 extends NewAutoBase {
    @Override
    protected Command getCommands() {
        TrajectorySequence traj1 = roadRunner.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(10,10), 0)
                .splineTo(new Vector2d(0, 20), 0)
                .build();
        return new RRDrive(roadRunner, traj1);
    }
}
