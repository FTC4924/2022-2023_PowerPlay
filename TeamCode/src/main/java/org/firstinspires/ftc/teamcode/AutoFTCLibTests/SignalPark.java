package org.firstinspires.ftc.teamcode.AutoFTCLibTests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.ftclib.RRDrive;
import org.firstinspires.ftc.teamcode.Commands.ftclib.SignalAction;
import org.firstinspires.ftc.teamcode.Commands.ftclib.SignalCamera;
import org.firstinspires.ftc.teamcode.NewAutoBase;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class SignalPark extends NewAutoBase {
    @Override
    protected Command getCommands() {
        Pose2d startPos = new Pose2d(-35.2,-70.8, Math.toRadians(90));
        setRoadRunnerStart(startPos);
        TrajectorySequence forwardTrajecory = roadRunner.trajectorySequenceBuilder(startPos)
                .strafeLeft(3)
                .forward(30)
                .build();

        TrajectorySequence leftTrajectory = roadRunner.trajectorySequenceBuilder(forwardTrajecory.end())
                .strafeLeft(25)
                .forward(10)
                .build();

        TrajectorySequence middleTrajectory = roadRunner.trajectorySequenceBuilder(forwardTrajecory.end())
                .forward(10)
                .build();

        TrajectorySequence rightTrajectory = roadRunner.trajectorySequenceBuilder(forwardTrajecory.end())
                .strafeRight(25)
                .forward(10)
                .build();
        return new SequentialCommandGroup(
                new SignalCamera(camera.getPipeline()),
                new RRDrive(roadRunner, forwardTrajecory),
                new SignalAction(
                        new RRDrive(roadRunner, leftTrajectory),
                        new RRDrive(roadRunner, middleTrajectory),
                        new RRDrive(roadRunner, rightTrajectory)
                )
        );
    }

}