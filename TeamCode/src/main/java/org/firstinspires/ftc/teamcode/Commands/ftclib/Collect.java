package org.firstinspires.ftc.teamcode.Commands.ftclib;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GripperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class Collect extends SequentialCommandGroup {
    public Collect(ArmSubsystem arm, LiftSubsystem lift, WristSubsystem wrist, GripperSubsystem gripper) {
        addCommands(
                new SequentialCommandGroup(
                        new Lift(lift, 0.95, 0.5),
                        new Gripper(gripper, Constants.GripperState.CLOSE, 1),
                        new ParallelCommandGroup(
                                new Arm(arm, Constants.ArmPos.ARM_COLLECTING, 0.5),
                                new Wrist(wrist, Constants.WristState.COLLECT, 1),
                                new Lift(lift, 0.25, 0.5)
                        ),
                        new Lift(lift, 0.0, 0.5)
                )
        );
    }
}
