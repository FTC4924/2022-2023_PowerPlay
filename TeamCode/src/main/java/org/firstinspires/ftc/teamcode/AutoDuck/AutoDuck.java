package org.firstinspires.ftc.teamcode.AutoDuck;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.DetectSignalSide;
import org.firstinspires.ftc.teamcode.Commands.Move;
import org.firstinspires.ftc.teamcode.Commands.Pause;

import java.util.ArrayList;
import java.util.Arrays;


public abstract class AutoDuck extends AutoBase {
    protected abstract AllianceColor getAllianceColor();
    protected ArrayList<Command> getCommands() {
        return new ArrayList<>(
                Arrays.asList(

                        new Pause(1),
                        new DetectSignalSide(),
                        new Move(.5, -45, 0.5)// Moves away from the wall
                )
        );
    }
}
