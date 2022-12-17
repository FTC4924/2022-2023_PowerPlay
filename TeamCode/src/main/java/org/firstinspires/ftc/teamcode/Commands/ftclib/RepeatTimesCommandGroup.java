package org.firstinspires.ftc.teamcode.Commands.ftclib;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.PerpetualCommand;

public class RepeatTimesCommandGroup extends PerpetualCommand {
    /**
     * Creates a new PerpetualCommand.  Will run another command in perpetuity, ignoring that
     * command's end conditions, unless this command itself is interrupted.
     *
     * @param command the command to run perpetually
     */
    public RepeatTimesCommandGroup(Command command) {
        super(command);
    }
}
