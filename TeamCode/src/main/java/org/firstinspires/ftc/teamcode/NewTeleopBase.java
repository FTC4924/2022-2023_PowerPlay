package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RoadRunnerSubsystem;

public class NewTeleopBase extends CommandOpMode {
    protected DriveSubsystem drive;
    protected ClawSubsystem gripper;
    protected ArmSubsystem arm;
    protected RoadRunnerSubsystem roadRunner;

    private GamepadEx gpad1;
    private GamepadEx gpad2;

    @Override
    public void initialize() {
        drive = new DriveSubsystem(
                hardwareMap,
                "leftFront",
                "rightFront",
                "leftBack",
                "rightBack"
        );

        gripper = new ClawSubsystem(hardwareMap, "gripper", "wrist");

        roadRunner = new RoadRunnerSubsystem(hardwareMap);

        arm = new ArmSubsystem(
                hardwareMap,
                "lift",
                "arm",
                "liftLimit",
                "null"  // TODO: 1/13/2023 Change for the addition of the arm limit switch
        );

        gpad1 = new GamepadEx(gamepad1);
        gpad2 = new GamepadEx(gamepad2);



        //schedule(new InstantCommand().andThen(getCommands()));

        register(drive, gripper, roadRunner, arm);
    }
}
