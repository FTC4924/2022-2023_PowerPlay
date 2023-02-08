package org.firstinspires.ftc.teamcode.Commands.ftclib;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Constants.ArmPos;
import org.firstinspires.ftc.teamcode.Constants.WristState;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GripperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class MiddleDeliver extends SequentialCommandGroup {

    private static final double INCHES = 3.0;
    private static final double SPEED = 0.5;


    public MiddleDeliver(ArmSubsystem arm, LiftSubsystem lift, WristSubsystem wrist, GripperSubsystem gripper) {
        super(
                new ParallelCommandGroup(
                        new Lift(lift, 0.25, 0.5),
                        new Wrist(wrist, WristState.SCORE, 1)
                ),
                new ParallelCommandGroup(
                        new Lift(lift, 0.5, 0.5),
                        new Arm(arm, ArmPos.ARM_SCORING, 0.5)
                )
        );
    }

}