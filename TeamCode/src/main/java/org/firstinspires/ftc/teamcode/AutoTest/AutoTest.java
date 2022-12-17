package org.firstinspires.ftc.teamcode.AutoTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Commands.preftclib.ArmRotate;
import org.firstinspires.ftc.teamcode.Commands.preftclib.Command;
import org.firstinspires.ftc.teamcode.Commands.preftclib.GripperGrab;
import org.firstinspires.ftc.teamcode.Commands.preftclib.LiftRaise;
import org.firstinspires.ftc.teamcode.Commands.preftclib.Move;
import org.firstinspires.ftc.teamcode.Commands.preftclib.Turn;

import java.util.ArrayList;
import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Constants.CHASSIS_RADIUS;

@Disabled
@Autonomous(name="AutoTest")
public class AutoTest extends AutoBase {
    protected AllianceColor getAllianceColor()  {
        return AllianceColor.BLUE;
    }
    protected boolean exitOnLastCommand = true;
    protected ArrayList<Command> getCommands() {
        return new ArrayList<>(
                Arrays.asList(
                        new Move(0.2, -90, 0.15),
                        new Move(3.85-CHASSIS_RADIUS, 0, 0.25),
                        new Turn(-159),
                        new LiftRaise(1.00),
                        new ArmRotate(1.00),
                        new LiftRaise(0.65),
                        new GripperGrab(1),
                        new LiftRaise(1),
                        new ArmRotate(0.80),
                        new GripperGrab(0),
                        new ArmRotate(0),
                        new LiftRaise(0),
                        new GripperGrab(0)

                )
        );
    }
}
