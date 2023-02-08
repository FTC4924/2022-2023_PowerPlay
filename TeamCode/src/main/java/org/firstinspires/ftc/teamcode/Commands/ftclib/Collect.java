package org.firstinspires.ftc.teamcode.Commands.ftclib;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Constants.ArmPos;
import org.firstinspires.ftc.teamcode.Constants.GripperState;
import org.firstinspires.ftc.teamcode.Constants.LiftPos;
import org.firstinspires.ftc.teamcode.Constants.WristState;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GripperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;




public class Collect extends SequentialCommandGroup {

    private static final double SPEED = 0.75;

    public Collect(ArmSubsystem arm, LiftSubsystem lift, WristSubsystem wrist, GripperSubsystem gripper) {
        addCommands(
                new SequentialCommandGroup(
                        new Lift(lift, LiftPos.LIFT_POLE_CLEAR, SPEED),
                        new Gripper(gripper, GripperState.CLOSE, 0.5),
                        new ParallelCommandGroup(
                                new Arm(arm, ArmPos.ARM_COLLECTING, SPEED),
                                new Wrist(wrist, WristState.COLLECT, 1),
                                new Lift(lift, LiftPos.LIFT_ARM_CLEAR, SPEED)
                        ),
                        new Gripper(gripper, GripperState.OPEN, 0.25),  // Gripper still moves after this command exits.
                        new Lift(lift, LiftPos.LIFT_COLLECT, SPEED)
                )
        );
    }
}
