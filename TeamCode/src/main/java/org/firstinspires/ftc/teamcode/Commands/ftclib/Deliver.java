package org.firstinspires.ftc.teamcode.Commands.ftclib;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Constants.LiftPos;
import org.firstinspires.ftc.teamcode.Constants.WristState;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GripperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class Deliver extends SequentialCommandGroup {

    private static final double INCHES = 3.0;
    private static final double SPEED = 0.5;


    public Deliver(ArmSubsystem arm, LiftSubsystem lift, WristSubsystem wrist, GripperSubsystem gripper) {
        addCommands(
                /*new ParallelCommandGroup(
                        new Lift(lift, 0.25, 0.5),
                        new Wrist(wrist, WristState.SCORE, 1)
                ),
                new ParallelCommandGroup(
                        new Lift(lift, 1, 0.5),
                        new Arm(arm, ArmPos.ARM_SCORING, 0.5)
                ),*/
                new Lift(lift, LiftPos.LIFT_SCORING, 0.5),
                new Wrist(wrist, WristState.SCORE, 0.5)/*,
                new Arm(arm, ArmPos.ARM_SCORING, 0.5)*/

        );
    }

}