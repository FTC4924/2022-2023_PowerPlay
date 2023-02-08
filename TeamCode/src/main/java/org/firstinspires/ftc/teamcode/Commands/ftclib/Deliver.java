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

public class Deliver extends SequentialCommandGroup {

//    private static final double INCHES = 3.0;
    private static final double SPEED = 1;


    public Deliver(ArmSubsystem arm, LiftSubsystem lift, WristSubsystem wrist, GripperSubsystem gripper) {
        super(
                new Gripper(gripper, GripperState.CLOSE, 0.5),
                new ParallelCommandGroup(
                        new Lift(lift, LiftPos.LIFT_ARM_CLEAR, SPEED),
                        new Wrist(wrist, WristState.SCORE, 1)
                ),
                new ParallelCommandGroup(
                        new Lift(lift, LiftPos.LIFT_SCORING, SPEED),
                        new Arm(arm, ArmPos.ARM_SCORING, SPEED)
                )
                /*Lift liftMax = new Lift(lift, 0.95, 0.5);
        addCommands(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new Lift(lift, 0.25, 0.5),
                                new ScheduleCommand(liftMax)
                        ),
                        new Wrist(wrist, WristState.SCORE, 1)
                ),
                new Arm(arm, ArmPos.ARM_SCORING, 0.5),
                new WaitUntilCommand(liftMax::isFinished)*/
                /*new Lift(lift, LiftPos.LIFT_SCORING, 0.5),
                new Wrist(wrist, WristState.SCORE, 0.5)/*,
                new Arm(arm, ArmPos.ARM_SCORING, 0.5)*/

        );
    }

}