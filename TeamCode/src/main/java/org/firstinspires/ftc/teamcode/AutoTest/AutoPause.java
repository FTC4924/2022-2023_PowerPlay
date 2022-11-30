package org.firstinspires.ftc.teamcode.AutoTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.Pause;

import java.util.ArrayList;
import java.util.Collections;

@Autonomous(name="AutoPause")
public class AutoPause extends AutoBase {
    protected AllianceColor getAllianceColor()  {
        return AllianceColor.BLUE;
    }
    protected boolean exitOnLastCommand = true;
    protected ArrayList<Command> getCommands() {
        return new ArrayList<Command>(
                Collections.singletonList(
                        new Pause(200)
                )
        );
    }
}
