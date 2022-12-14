package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive extends SubsystemBase { // TODO: 12/13/2022 Rewrite to use RoadRunner.

    private final MotorEx frontLeft, frontRight, backLeft, backRight;

    private final HDrive xDrive;

    public Drive(MotorEx frontLeft, MotorEx frontRight, MotorEx backLeft, MotorEx backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        this.xDrive = new HDrive(frontLeft, frontRight, backLeft, backRight);
    }

    public Drive(HardwareMap hMap, String frontLeft, String frontRight, String backLeft, String backRight) {
        this(
                new MotorEx(hMap, frontLeft),
                new MotorEx(hMap, frontRight),
                new MotorEx(hMap, backLeft),
                new MotorEx(hMap, backRight)
        );
    }

    public void drive(double strafeSpeed, double forwardSpeed, double turn, double heading) {
        xDrive.driveFieldCentric(strafeSpeed, forwardSpeed, turn, heading);
    }
}
