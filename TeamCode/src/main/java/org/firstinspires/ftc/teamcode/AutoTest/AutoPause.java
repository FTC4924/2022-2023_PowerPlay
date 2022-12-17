package org.firstinspires.ftc.teamcode.AutoTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Commands.preftclib.Command;
import org.firstinspires.ftc.teamcode.Commands.preftclib.Pause;

import java.util.ArrayList;
import java.util.Collections;

@Disabled
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
