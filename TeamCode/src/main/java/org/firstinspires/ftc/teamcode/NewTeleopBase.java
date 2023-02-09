package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Commands.ftclib.Arm;
import org.firstinspires.ftc.teamcode.Commands.ftclib.Collect;
import org.firstinspires.ftc.teamcode.Commands.ftclib.Deliver;
import org.firstinspires.ftc.teamcode.Commands.ftclib.Lift;
import org.firstinspires.ftc.teamcode.Commands.ftclib.TelemetryCommand;
import org.firstinspires.ftc.teamcode.Commands.ftclib.defaultcommands.DefaultArm;
import org.firstinspires.ftc.teamcode.Commands.ftclib.defaultcommands.DefaultDrive;
import org.firstinspires.ftc.teamcode.Commands.ftclib.defaultcommands.DefaultLift;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GripperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;
import org.firstinspires.ftc.teamcode.triggers.AxisTrigger;
import org.firstinspires.ftc.teamcode.triggers.JoystickTrigger;

import static org.firstinspires.ftc.teamcode.Constants.ANALOG_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.ArmPos;
import static org.firstinspires.ftc.teamcode.Constants.LiftPos;

public abstract class NewTeleopBase extends CommandOpMode {
    protected DriveSubsystem drive;
    protected GripperSubsystem gripper;
    protected WristSubsystem wrist;
    protected ArmSubsystem arm;
    protected LiftSubsystem lift;
    //protected RoadRunnerSubsystem roadRunner;

    private Deliver deliverCommand;
    private Collect collectCommand;

    private GamepadEx gpad1;
    private GamepadEx gpad2;

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

        gripper = new GripperSubsystem(hardwareMap, "gripper");

        wrist = new WristSubsystem(hardwareMap, "wrist");

        //roadRunner = new RoadRunnerSubsystem(hardwareMap);

        arm = new ArmSubsystem(
                hardwareMap,
                "arm",
                "armLimit"
        );

        lift = new LiftSubsystem(
                hardwareMap,
                "lift",
                "liftLimit"
        );

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


        Command wristToggle = new ConditionalCommand(
                new InstantCommand(wrist::wristScore, wrist),
                new InstantCommand(wrist::wristCollect, wrist),
                () -> wrist.getPos() >= 0.5
        );

        Command armScoring = new Arm(arm, ArmPos.ARM_SCORING, 0.75);
        Command armCollect = new Arm(arm, ArmPos.ARM_COLLECTING, 0.75);
        Command armManual = new DefaultArm(arm, gpad2::getRightY);
        Command stopArm = new InstantCommand(arm::stop, arm);

        Command liftUp = new Lift(lift, LiftPos.LIFT_MAX, 0.75);
        Command liftDown = new Lift(lift, LiftPos.LIFT_MIN, 0.75);
        Command liftManual = new DefaultLift(lift, gpad2::getLeftY, gpad2.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)::get, telemetry);
        Command stopLift = new InstantCommand(lift::stop, lift);
        Command zeroLift = new InstantCommand(lift::zero, lift);

        Command driveCommand = new DefaultDrive(drive, gpad1::getLeftX, gpad1::getLeftY, this::getGpad1LeftTrigger, this::getGpad1RightTrigger, gpad1.getGamepadButton(GamepadKeys.Button.Y)::get);
        Command resetGyro = new InstantCommand(drive::resetGyro);

        deliverCommand = new Deliver(arm, lift, wrist, gripper);
        collectCommand = new Collect(arm, lift, wrist, gripper);


        ///////////////////////////// Gamepad 1 keybindings /////////////////////////////
        /*gpad1LeftStick
                .or(gpad1LeftTrigger)
                .or(gpad1RightTrigger)
                .whileActiveContinuous(driveCommand)
                .whenInactive(new InstantCommand(drive::stop, drive));  // TODO: 1/26/23 Check that this works with RoadRunner before using RoadRunner.
*/
        gpad1.getGamepadButton(GamepadKeys.Button.B)  // Reset the Gyro
                .whenActive(resetGyro);


        ///////////////////////////// Gamepad 2 keybindings /////////////////////////////

        /*gpad2LeftStick.y.and(stateManual).whenInactive(stopLift);  // Stop the Lift.


        gpad2RightStick.y
                .and(stateManual)
                .whileActiveContinuous(armManual)  // Actuate the Arm
                .whenInactive(stopArm);*/

        gpad2.getGamepadButton(GamepadKeys.Button.X)
                .whenInactive(this::cancelAll);  // Toggle the active state

        gpad2.getGamepadButton(GamepadKeys.Button.Y)  // Actuate the Gripper
                .or(gpad2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER))
                .or(gpad2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER))
                .toggleWhenActive(
                        new SequentialCommandGroup(
                                new InstantCommand(gripper::close, gripper),
                                new TelemetryCommand(telemetry, "Gripper Close", 1)
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(gripper::open, gripper),
                                new TelemetryCommand(telemetry, "Gripper Open", 1)
                        )
                );

        gpad2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)  // Actuate the Wrist
                .whenActive(wristToggle);

        gpad2.getGamepadButton(GamepadKeys.Button.DPAD_UP)  // Zero the Lift
                .and(gpad2RightTrigger)
                .whenActive(zeroLift);


        gpad2.getGamepadButton(GamepadKeys.Button.A)  // Zero the Lift
                .whenActive(deliverCommand);

        gpad2.getGamepadButton(GamepadKeys.Button.B)
                .whenActive(collectCommand);


        register(drive, gripper, wrist, arm, lift);  /*roadRunner,*/

        ///////////////////////////// Subsystem Default Commands /////////////////////////////

        lift.setDefaultCommand(liftManual);

        arm.setDefaultCommand(armManual);

        drive.setDefaultCommand(driveCommand);
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("Manual State", gpad2.getGamepadButton(GamepadKeys.Button.Y)  // Actuate the Gripper
                .or(gpad2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER))
                .get()
        );
        telemetry.addData("Lift Pos", lift.getPos());
        telemetry.addData("Arm Pos", arm.getPos());
        telemetry.addData("Arm Offset", arm.getOffset());
        telemetry.addData("Arm Pressed", arm.pressed());
        telemetry.update();
    }

    private void telem() {
        telemetry.addData("telem method", "");
    }

    /*private void armM() {
        if (gpad2.getRightY() > 0) arm.setPos(ArmPos.ARM_SCORING);
        else arm.setPos(ArmPos.ARM_COLLECTING);

        arm.setPower(gpad2.getRightY());
    }*/

    /*private void drive() {
        drive.drive(
                gpad1.getLeftX() / 2,
                gpad1.getLeftY() / 2,
                (gpad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - gpad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) / 2
        );
    }*/

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

    private void cancelAll() {
        CommandScheduler.getInstance().cancel(deliverCommand, collectCommand);
    }

    protected abstract AllianceColor getAlliance();

}
