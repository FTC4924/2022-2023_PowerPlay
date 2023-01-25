package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RoadRunnerSubsystem;

import static org.firstinspires.ftc.teamcode.Constants.TELEOP_STATE;

public abstract class NewTeleopBase extends CommandOpMode {
    protected DriveSubsystem drive;
    protected ClawSubsystem gripper;
    protected ArmSubsystem arm;
    protected RoadRunnerSubsystem roadRunner;

    private GamepadEx gpad1;
    private GamepadEx gpad2;

    private TELEOP_STATE teleopState;

    private Trigger stateManual;
    private Trigger stateAuto;
    private Trigger stateTest;

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

        teleopState = TELEOP_STATE.MANUAL;

        // TODO: 1/24/2023 Instantiate Commands for buttons here
        Command clawToggle = new ConditionalCommand(
                new InstantCommand(gripper::gripperClose),
                new InstantCommand(gripper::gripperOpen),
                () -> gripper.getGripperPos() >= 0.5
        );

        Command wristToggle = new ConditionalCommand(
                new InstantCommand(gripper::wristScore),
                new InstantCommand(gripper::wristCollect),
                () -> gripper.getWristPos() >= 0.5
        );




        gpad2.getGamepadButton(GamepadKeys.Button.Y)
                .or(gpad2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER))
                .and(stateManual)
                .whenActive(clawToggle);

        gpad2.getGamepadButton(GamepadKeys.Button.B)
                .or(gpad2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER))
                .and(stateManual)
                .whenActive(wristToggle);

        gpad1.getGamepadButton(GamepadKeys.Button.A).and(stateManual).whenActive(clawToggle);


        ///////////////////////////// Gamepad 1 keybindings /////////////////////////////

       // gpad1.getGamepadButton(GamepadKeys.Trigger.values());

        gpad2.getGamepadButton(GamepadKeys.Button.X).whenInactive(this::configureState);  // Change the state on gpad 2 button x regardless of state.



        //schedule(new InstantCommand().andThen(getCommands()));

        register(drive, gripper, roadRunner, arm);
    }

    private void armScoring() {
        arm.setArm(1.0);
    }
    private void armCollect() {
        arm.setArm(0.0);
        }
    private void armM() {
        arm.setArm(gpad2.gamepad.right_stick_y);
    }

    private void configureState() {
        switch (teleopState) {
            case MANUAL:
                teleopState = TELEOP_STATE.AUTO;
                break;
            case AUTO:
                teleopState = TELEOP_STATE.MANUAL;
                break;
            case TEST:
                teleopState = TELEOP_STATE.MANUAL;
                break;
        }
    }
}
