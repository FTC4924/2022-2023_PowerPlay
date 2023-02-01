package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GripperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RoadRunnerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public abstract class NewAutoBase extends CommandOpMode {
    protected DriveSubsystem drive;
    protected GripperSubsystem gripper;
    protected WristSubsystem wrist;
    protected ArmSubsystem arm;
    protected LiftSubsystem lift;
    protected RoadRunnerSubsystem roadRunner;
    protected Camera camera;
    @Override
    public void initialize() {
        drive = new DriveSubsystem(
                hardwareMap,
                "leftFront",
                "rightFront",
                "leftBack",
                "rightBack"
        );

        gripper = new GripperSubsystem(hardwareMap, "gripper");

        roadRunner = new RoadRunnerSubsystem(hardwareMap);

        arm = new ArmSubsystem(
                hardwareMap,
                "arm",
                "null"  // TODO: 1/13/2023 Change for the addition of the arm limit switch
        );
        
        lift = new LiftSubsystem(
                hardwareMap,
                "lift",
                "liftLimit"
        );

        camera = new Camera(hardwareMap, telemetry);

        schedule(new InstantCommand().andThen(getCommands()));

        register(drive, gripper, wrist, roadRunner, arm, lift);
    }
    
    public void setRoadRunnerStart(Pose2d pose2d) {
        roadRunner.setPoseEstimate(pose2d);
    }

    protected abstract Command getCommands();
}
