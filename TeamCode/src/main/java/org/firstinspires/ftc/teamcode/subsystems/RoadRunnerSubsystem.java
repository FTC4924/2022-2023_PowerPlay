package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

public class RoadRunnerSubsystem extends SubsystemBase {
    private SampleMecanumDriveCancelable drive;
    private Pose2d lastTrajectoryEndPose;

    public RoadRunnerSubsystem(HardwareMap hardwareMap, Pose2d startPose) {
        drive = new SampleMecanumDriveCancelable(hardwareMap);
        lastTrajectoryEndPose = startPose;
    }

    @Override
    public void periodic() {
        drive.update();
        PoseStorage.pose2d = drive.getPoseEstimate();
    }

    public boolean isBusy() {
        return drive.isBusy();
    }

    public void followTrajectorySequence(TrajectorySequence trajectory) {
        drive.followTrajectorySequenceAsync(trajectory);
    }

    public void turn(double angle) {
        drive.turnAsync(angle);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder() {
        return drive.trajectorySequenceBuilder(null);
    }

}
