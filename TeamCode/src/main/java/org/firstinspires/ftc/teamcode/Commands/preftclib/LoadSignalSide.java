package org.firstinspires.ftc.teamcode.Commands.preftclib;

import java.util.ArrayList;

/**
 * Command that runs one set of commands based on the result of a previous call of
 * <a href="#{@link}">{@link DetectSignalSide}</a>
 * @see DetectSignalSide
 */
public class LoadSignalSide extends Command {
    public LoadSignalSide(ArrayList<Command> leftCommands, ArrayList<Command> centerCommands, ArrayList<Command> rightCommands) {
        this.leftCommands = leftCommands;
        this.centerCommands = centerCommands;
        this.rightCommands = rightCommands;
    }
}
