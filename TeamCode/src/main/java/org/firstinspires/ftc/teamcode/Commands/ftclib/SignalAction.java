package org.firstinspires.ftc.teamcode.Commands.ftclib;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SelectCommand;

import org.firstinspires.ftc.teamcode.Constants;

import java.util.HashMap;

public class SignalAction extends SelectCommand {

    public SignalAction(Command leftCommand, Command middleCommand, Command rightCommand) {
        super(
                new HashMap<Object, Command>() {{
                    put(Constants.SignalSide.SIDE_R, leftCommand);
                    put(Constants.SignalSide.SIDE_B, middleCommand);
                    put(Constants.SignalSide.SIDE_P, rightCommand);
                }},
                // the selector
                SignalCamera::getSignalSide
        );

    }
}
