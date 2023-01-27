package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.ANALOG_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.ArmPos;
import static org.firstinspires.ftc.teamcode.Constants.LiftPos;
import static org.firstinspires.ftc.teamcode.Constants.TELEOP_STATE;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Commands.ftclib.Arm;
import org.firstinspires.ftc.teamcode.Commands.ftclib.Lift;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.triggers.AxisTrigger;
import org.firstinspires.ftc.teamcode.triggers.JoystickTrigger;
import org.firstinspires.ftc.teamcode.triggers.TeleopStateTrigger;

public abstract class NewTeleopBase extends CommandOpMode {
    protected DriveSubsystem drive;
    protected ClawSubsystem gripper;
    protected ArmSubsystem arm;
    //protected RoadRunnerSubsystem roadRunner;

    private GamepadEx gpad1;
    private GamepadEx gpad2;

    private TELEOP_STATE teleopState;

    @SuppressWarnings("FieldCanBeLocal")
    private AllianceColor alliance;

    @Override
    public void initialize() {
        alliance = getAlliance();

        drive = new DriveSubsystem(
                hardwareMap,
                "leftFront",
                "rightFront",
                "leftBack",
                "rightBack"
        );

        gripper = new ClawSubsystem(hardwareMap, "gripper", "wrist");

        //roadRunner = new RoadRunnerSubsystem(hardwareMap);

        arm = new ArmSubsystem(
                hardwareMap,
                "lift",
                "arm",
                "liftLimit",
                "null"  // TODO: 1/13/2023 Change for the addition of the arm limit switch
        );

        teleopState = TELEOP_STATE.MANUAL;

        Trigger stateManual = new TeleopStateTrigger(this::getTeleopState, TELEOP_STATE.MANUAL);
        Trigger stateAuto = new TeleopStateTrigger(this::getTeleopState, TELEOP_STATE.AUTO);
        Trigger stateTest = new TeleopStateTrigger(this::getTeleopState, TELEOP_STATE.TEST);

        gpad1 = new GamepadEx(gamepad1);
        gpad2 = new GamepadEx(gamepad2);

        JoystickTrigger gpad1LeftStick = new JoystickTrigger(gpad1::getLeftX, gpad1::getLeftY);
        JoystickTrigger gpad1RightStick = new JoystickTrigger(gpad1::getRightX, gpad1::getRightY);
        JoystickTrigger gpad2LeftStick = new JoystickTrigger(gpad2::getLeftX, gpad2::getLeftY);
        JoystickTrigger gpad2RightStick = new JoystickTrigger(gpad2::getRightX, gpad2::getRightY);

        AxisTrigger gpad1LeftTrigger = new AxisTrigger(this::getGpad1LeftTrigger, ANALOG_THRESHOLD);
        AxisTrigger gpad1RightTrigger = new AxisTrigger(this::getGpad1RightTrigger, ANALOG_THRESHOLD);
        AxisTrigger gpad2LeftTrigger = new AxisTrigger(this::getGpad2LeftTrigger, ANALOG_THRESHOLD);
        AxisTrigger gpad2RightTrigger = new AxisTrigger(this::getGpad2RightTrigger, ANALOG_THRESHOLD);

        // TODO: 1/24/2023 Instantiate Commands for buttons here
        Command gripperToggle = new ConditionalCommand(
                new InstantCommand(gripper::gripperClose, gripper),
                new InstantCommand(gripper::gripperOpen, gripper),
                () -> gripper.getGripperPos() >= 0.5
        );

        Command wristToggle = new ConditionalCommand(
                new InstantCommand(gripper::wristScore, gripper),
                new InstantCommand(gripper::wristCollect, gripper),
                () -> gripper.getWristPos() >= 0.5
        );

        Command armScoring = new Arm(arm, ArmPos.ARM_SCORING, 0.75);
        Command armCollect = new Arm(arm, ArmPos.ARM_COLLECTING, 0.75);
        Command armManual = new InstantCommand(this::armM, arm);
        Command stopArm = new InstantCommand(arm::stopArm, arm);

        Command liftUp = new Lift(arm, LiftPos.LIFT_UP, 0.75);
        Command liftDown = new Lift(arm, LiftPos.LIFT_DOWN, 0.75);
        Command liftManual = new InstantCommand(this::liftM, arm);
        Command liftOverride = new InstantCommand(this::liftO, arm);
        Command stopLift = new InstantCommand(arm::stopLift, arm);
        Command zeroLift = new InstantCommand(arm::zeroLift, arm);

        Command driveCommand = new InstantCommand(this::drive, drive);
        Command resetGyro = new InstantCommand(drive::resetGyro);


        ///////////////////////////// Gamepad 1 keybindings /////////////////////////////
        gpad1LeftStick.whileActiveContinuous(driveCommand);  // TODO: 1/26/23 Check that this works with RoadRunner before using RoadRunner.

        gpad1.getGamepadButton(GamepadKeys.Button.B)  // Reset the Gyro
                .whenActive(resetGyro);


        ///////////////////////////// Gamepad 2 keybindings /////////////////////////////
        gpad2LeftStick.y  // Actuate the Lift
                .and(gpad2.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).negate())
                .and(stateManual)
                .whileActiveContinuous(liftManual);

        gpad2LeftStick.y  // Actuate the Lift if overridden by the right stick button
                .and(gpad2.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON))
                .and(stateManual)
                .whileActiveContinuous(liftOverride);

        gpad2LeftStick.y.and(stateManual).whenInactive(stopLift);  // Stop the Lift.


        gpad2RightStick.y
                .and(stateManual)
                .whileActiveContinuous(armManual)  // Actuate the Arm
                .whenInactive(stopArm);

        gpad2.getGamepadButton(GamepadKeys.Button.X)
                .and(stateManual)
                .whenInactive(this::configureState);  // Toggle the active state

        gpad2.getGamepadButton(GamepadKeys.Button.Y)  // Actuate the Gripper
                .or(gpad2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER))
                .and(stateManual)
                .whenActive(gripperToggle);

        gpad2.getGamepadButton(GamepadKeys.Button.B)  // Actuate the Wrist
                .or(gpad2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER))
                .and(stateManual)
                .whenActive(wristToggle);

        gpad2.getGamepadButton(GamepadKeys.Button.DPAD_UP)  // Zero the Lift
                .and(gpad2RightTrigger)
                .and(stateManual)
                .whenActive(zeroLift);


        register(drive, gripper, /*roadRunner,*/ arm);
    }

    private void armM() {
        if (gpad2.getRightY() > 0) arm.setArm(ArmPos.ARM_SCORING);
        else arm.setArm(ArmPos.ARM_COLLECTING);

        arm.setArmPower(gpad2.getRightY());
    }

    private void liftM() {
        arm.setLiftOverride(true);

        if (gpad2.getLeftY() > 0) arm.setLift(LiftPos.LIFT_UP);
        else arm.setLift(LiftPos.LIFT_DOWN);

        arm.setLiftPower(gpad2.getLeftY());
    }

    private void liftO() {
        arm.setLiftOverride(false);
        arm.setLift(arm.getLiftPos() + (int) (gpad2.getLeftY() * 100));
        arm.setLiftPower(gpad2.getLeftY());
    }

    private void drive() {
        drive.drive(
                gpad1.getLeftX(),
                gpad1.getLeftY(),
                gpad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - gpad2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
        );
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

    private TELEOP_STATE getTeleopState() {
        return teleopState;
    }

    private double getGpad1LeftTrigger() {
        return gpad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
    }

    private double getGpad1RightTrigger() {
        return gpad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
    }

    private double getGpad2LeftTrigger() {
        return gpad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
    }

    private double getGpad2RightTrigger() {
        return gpad2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
    }

    protected abstract AllianceColor getAlliance();


}
