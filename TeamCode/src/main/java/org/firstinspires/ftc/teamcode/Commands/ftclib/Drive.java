package org.firstinspires.ftc.teamcode.Commands.ftclib;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing.Timer;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.concurrent.TimeUnit;

public class Drive extends CommandBase {

    private final DriveSubsystem m_drive;
    private final double m_forward;
    private final double m_rotation;
    private final double m_strafe;
    private final Timer timer;

    /**
     * Creates a new DriveCommand.
     *  @param subsystem The drive subsystem this command will run on.
     * @param forward The control input for driving forwards/backwards
     * @param strafe The control input for driving sideways.
     * @param rotation The control input for turning
     * @param duration The duration to move in seconds.
     */
    public Drive(DriveSubsystem subsystem, double forward,
                 double strafe, double rotation, double duration) {
        m_drive = subsystem;
        m_forward = forward;
        m_rotation = rotation;
        m_strafe = strafe;
        timer = new Timer((int) (duration * 1000000), TimeUnit.MICROSECONDS);
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_drive.drive(
                m_forward,
                m_rotation,
                m_strafe
        );
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return timer.done();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }
}
