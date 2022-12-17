package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

public class DriveSubsystem extends SubsystemBase { // TODO: 12/13/2022 Rewrite to use RoadRunner.

    private final MotorEx frontLeft, frontRight, backLeft, backRight;

    private final HDrive xDrive;

    private final BNO055IMU imu;
    private Orientation angles;

    public DriveSubsystem(MotorEx frontLeft, MotorEx frontRight, MotorEx backLeft, MotorEx backRight, BNO055IMU imu) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        this.xDrive = new HDrive(frontLeft, frontRight, backLeft, backRight);

        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);
        this.imu = imu;
        angles = null;
    }

    public DriveSubsystem(HardwareMap hMap, String frontLeft, String frontRight, String backLeft, String backRight) {
        this(
                new MotorEx(hMap, frontLeft),
                new MotorEx(hMap, frontRight),
                new MotorEx(hMap, backLeft),
                new MotorEx(hMap, backRight),
                hMap.get(BNO055IMU.class, "imu")
        );
    }

    public void drive(double strafeSpeed, double forwardSpeed, double turn) {
        double heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, RADIANS).firstAngle;
        xDrive.driveFieldCentric(strafeSpeed, forwardSpeed, turn, heading);
    }

    public void stop() { xDrive.stop(); }
}
