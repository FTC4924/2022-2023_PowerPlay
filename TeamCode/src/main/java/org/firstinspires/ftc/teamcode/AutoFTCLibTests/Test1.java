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
        setRoadRunnerStart(new Pose2d(-35.2,-70.8, Math.toRadians(90)));
        TrajectorySequence traj1 = roadRunner.trajectorySequenceBuilder( new Pose2d(-35.2,-70.8, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(-35.2,-35.6), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-13.6,-35.6), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-13.2,-23.6), Math.toRadians(0))
                .build();
        return new RRDrive(roadRunner, traj1);
    }

}