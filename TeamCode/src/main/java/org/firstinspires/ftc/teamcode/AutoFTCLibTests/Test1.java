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
        TrajectorySequence traj1 = roadRunner.trajectorySequenceBuilder(new Pose2d(-32, -62, Math.toRadians(90)))
                //.setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(-45,-35), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-20,-35), Math.toRadians(90))
                .build();
        return new RRDrive(roadRunner, traj1);
    }

}