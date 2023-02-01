package org.firstinspires.ftc.teamcode.AutoFTCLibTests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.ftclib.RRDrive;
import org.firstinspires.ftc.teamcode.NewAutoBase;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class Park extends NewAutoBase {
    @Override
    protected Command getCommands() {
        Pose2d startPos = new Pose2d(-35.2,-70.8, Math.toRadians(90));
        setRoadRunnerStart(startPos);
        TrajectorySequence forwardTrajecory = roadRunner.trajectorySequenceBuilder(startPos)
                .strafeLeft(3)
                .forward(35)
                .build();

        return new SequentialCommandGroup(
                new RRDrive(roadRunner, forwardTrajecory)
        );
    }

}