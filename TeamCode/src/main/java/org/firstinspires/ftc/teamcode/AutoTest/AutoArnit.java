package org.firstinspires.ftc.teamcode.AutoTest;

import static org.firstinspires.ftc.teamcode.Constants.CHASSIS_RADIUS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Commands.ArmRotate;
import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.DetectSignalSide;
import org.firstinspires.ftc.teamcode.Commands.GripperGrab;
import org.firstinspires.ftc.teamcode.Commands.LiftRaise;
import org.firstinspires.ftc.teamcode.Commands.LoadSignalSide;
import org.firstinspires.ftc.teamcode.Commands.Move;
import org.firstinspires.ftc.teamcode.Commands.Pause;
import org.firstinspires.ftc.teamcode.Commands.Turn;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

@Autonomous(name = "AutoArnit")
public class AutoArnit extends AutoBase {
    protected AllianceColor getAllianceColor() {
        return AllianceColor.BLUE;
    }

    protected boolean exitOnLastCommand = true;

    protected ArrayList<Command> getCommands() {
        return new ArrayList<Command>(
                Arrays.asList(

                        new Pause(0.125),
                        new DetectSignalSide(),
                        new Move(2.9 - CHASSIS_RADIUS, 0.4),
                        new Pause(0.25),
                        new LoadSignalSide(
                                new ArrayList<>(
                                        Collections.singletonList(
                                                new Move(1.9, 90, 0.4)
                                        )
                                ),
                                new ArrayList<>(
                                        Collections.emptyList(

                                        )
                                ),
                                new ArrayList<>(
                                        Collections.singletonList(
                                                new Move(1.75, -90, 0.4)
                                        )
                                )
                        ),
                        new Pause(0.25),
                        new Move(1, 0.4),
                        new LoadSignalSide(
                                new ArrayList<>(),
                                new ArrayList<>(
                                        Arrays.asList(
                                                new ArmRotate(.80),
                                                new LiftRaise(.85),
                                                new Turn(-30),
                                                new LiftRaise(0.80),
                                                new GripperGrab(1.0),
                                                new Pause(1),
                                                new ArmRotate(.65),
                                                new GripperGrab(0),
                                                new ArmRotate(0),
                                                new LiftRaise(0)
                                        )
                                ),
                                new ArrayList<>()
                        ),
                        new Pause(0.25)
                )
        );
    }
}