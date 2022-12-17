package org.firstinspires.ftc.teamcode.AutoTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Commands.preftclib.Command;
import org.firstinspires.ftc.teamcode.Commands.preftclib.DetectSignalSide;
import org.firstinspires.ftc.teamcode.Commands.preftclib.LoadSignalSide;
import org.firstinspires.ftc.teamcode.Commands.preftclib.Move;
import org.firstinspires.ftc.teamcode.Commands.preftclib.Pause;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

import static org.firstinspires.ftc.teamcode.Constants.CHASSIS_RADIUS;

@Autonomous(name="AutoSignalPark")
public class AutoSignalPark extends AutoBase {
    protected AllianceColor getAllianceColor() {
        return AllianceColor.BLUE;
    }
    protected boolean exitOnLastCommand = true;
    protected ArrayList<Command> getCommands() {
        return new ArrayList<Command>(
                Arrays.asList(
                        new Pause(0.125),
                        new DetectSignalSide(),
                        new Move(2.9 - CHASSIS_RADIUS,0.4),
                        new Pause(0.25),
                        new LoadSignalSide(
                                new ArrayList<>(
                                        Collections.singletonList(
                                                new Move(1.9, 90, 0.4)
                                        )
                                ),
                                new ArrayList<>(
                                        Collections.emptyList()
                                ),
                                new ArrayList<>(
                                        Collections.singletonList(
                                                new Move(1.75, -90, 0.4)
                                        )
                                )
                        ),
                        new Pause(0.25),
                        new Move(1,0.4),
                        new Pause(0.25)
                )
        );
    }
}
