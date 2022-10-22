package org.firstinspires.ftc.teamcode;

import android.os.Build;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import androidx.annotation.RequiresApi;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.teamcode.Constants.ARM_ROTATOR_SPEED;
import static org.firstinspires.ftc.teamcode.Constants.CLAW_GRABBER_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.CLAW_GRABBER_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.CLAW_ROTATOR_COLLECTING_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.CLAW_ROTATOR_SCORING_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.JOYSTICK_TOLERANCE;
import static org.firstinspires.ftc.teamcode.Constants.SPEED;
import static org.firstinspires.ftc.teamcode.Constants.TURNING_POWER_SCALAR;

public abstract class TeleopBase extends OpMode {

    protected static AllianceColor allianceColor;

    // Motors
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor armRotator;
    private DcMotor armRaiser;
    private Servo clawRotator;
    private Servo clawGrabber;


    // Gyro sensor
    private BNO055IMU imu;

    private Orientation angles;
    private double angleOffset;
    private double currentRobotAngle;

    // Toggle booleans
    private boolean yPressed;
    private boolean aPressed;

    private boolean clawGrabberOpen;
    private boolean clawRotatorScoring;

    public void init() {

        allianceColor = getAllianceColor();

        /*Instantiating the motor and servo objects as their appropriate motor/servo in the
        configuration on the robot*/
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        armRotator = hardwareMap.get(DcMotor.class, "armRotator");
        armRaiser = hardwareMap.get(DcMotor.class, "armRaiser");
        clawRotator = hardwareMap.get(Servo.class, "clawRotator");
        clawGrabber = hardwareMap.get(Servo.class, "clawGrabber");

        armRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRaiser.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRaiser.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRaiser.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawGrabberOpen = false;
        clawRotatorScoring = false;


        // Initializing the RevHub IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles = null;
        currentRobotAngle = 0.0;
        angleOffset = allianceColor.angleOffset;

    }

    public void start() {
        resetRuntime();
    }

    @RequiresApi(api = Build.VERSION_CODES.O)
    public void loop() {

        getAngle();

        recalibrateGyro();

        holonomicDrive();

        arm();

        claw();

        telemetry.addData( "getCurrentPosition", armRaiser.getCurrentPosition());

    }

    /**
     * Gets the angle of the robot from the rev imu and subtracts the angle offset.
     */
    private void getAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, RADIANS);
        currentRobotAngle = angles.firstAngle - angleOffset;
    }

    /**
     * Sets the gyro angle offset based off of the current angle when the b button is pressed.
     */
    private void recalibrateGyro() {
        if(gamepad1.b) {
            angleOffset = angles.firstAngle;
        }
    }

    /**
     * Holonomic controls according to what direction the robot is facing when we start the
     * program or when we recalibrate the gyro.
     * Uses the left stick to control movement, the triggers to control turning using exponential
     * controls, and the right stick up and down for speed.
     */
    private void holonomicDrive() {

        double gamepad1LeftStickX = gamepad1.left_stick_x;
        double gamepad1LeftStickY = gamepad1.left_stick_y;
        double gamepad1LeftTrigger = gamepad1.left_trigger;
        double gamepad1RightTrigger = gamepad1.right_trigger;

        double angleError = 0.0;

        double leftFrontPower;
        double leftBackPower ;
        double rightFrontPower;
        double rightBackPower;

        if (Math.abs(gamepad1LeftStickX) >= JOYSTICK_TOLERANCE || Math.abs(gamepad1LeftStickY) >= JOYSTICK_TOLERANCE) {

            // Uses atan2 to convert the x and y values of the controller to an angle
            double gamepad1LeftStickAngle = Math.atan2(gamepad1LeftStickY, gamepad1LeftStickX);

            /*Subtracts the robot's current angle from the command angle so that it travels globally
            rather than relative to the robot, then rotates it 45 degrees so that the angle's components
            align with the wheels*/
            double holonomicAngle = gamepad1LeftStickAngle + currentRobotAngle + Math.PI / 4;

            // overall power based on how far the stick is from the center
            double power = Math.sqrt(Math.pow(gamepad1LeftStickX, 2) + Math.pow(gamepad1LeftStickY, 2)) * SPEED;

            // the main diagonal is the diagonal from top left to bottom right
            double mainDiagonalPercent = Math.cos(holonomicAngle);
            // the anti-diagonal is the diagonal from topRight to bottomLeft
            double antiDiagonalPercent = Math.sin(holonomicAngle);

            leftFrontPower = mainDiagonalPercent * -power;
            rightBackPower = mainDiagonalPercent * power;
            rightFrontPower = antiDiagonalPercent * -power;
            leftBackPower = antiDiagonalPercent * power;

        } else {

            leftFrontPower = 0.0;
            leftBackPower = 0.0;
            rightFrontPower = 0.0;
            rightBackPower = 0.0;

        }

        if (gamepad1LeftTrigger >= JOYSTICK_TOLERANCE) {
            angleError += Math.pow(gamepad1LeftTrigger, 2);
        }
        if (gamepad1RightTrigger >= JOYSTICK_TOLERANCE) {
            angleError -= Math.pow(gamepad1RightTrigger, 2);
        }

        leftFrontPower += angleError * TURNING_POWER_SCALAR;
        leftBackPower += angleError * TURNING_POWER_SCALAR;
        rightFrontPower += angleError * TURNING_POWER_SCALAR;
        rightBackPower += angleError * TURNING_POWER_SCALAR;

        // Sets the wheel powers
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }

    /**
     * Arm Controls
     */
    private void arm() {

        /*
        Linearly raises and lowers the arm :
         */
        if (Math.abs(gamepad2.left_stick_y) >= JOYSTICK_TOLERANCE) {
            armRaiser.setPower(gamepad2.left_stick_y*-1);
        } else {
            armRaiser.setPower(0.0);

        }

        /*
        Rotates the arm
         */
        if (Math.abs(gamepad2.right_stick_x) >= JOYSTICK_TOLERANCE) {
            armRotator.setPower(gamepad2.right_stick_x/ARM_ROTATOR_SPEED);
            telemetry.addData("armRotator", true);
        } else {
            armRotator.setPower(0.0);
            telemetry.addData("armRotator", false);

        }
    }

    /**
     * Claw Controls
     */
    private void claw() {
        /*
        Toggle for claw grabber (open and close)
         */
        if (gamepad2.y && !yPressed) {
            yPressed = true;
            telemetry.addData("clawGrabber", true);
            if (clawGrabberOpen) {
                clawGrabberOpen = false;
                clawGrabber.setPosition(CLAW_GRABBER_CLOSE_POSITION);
            } else {
                clawGrabberOpen = true;
                clawGrabber.setPosition((CLAW_GRABBER_OPEN_POSITION));
            }
        } else if (!gamepad2.y) {
            yPressed = false;
            telemetry.addData("clawGrabber", false);

        }

        /*
        Claw Rotator Scoring
         */
        if (gamepad2.a && !aPressed) {
            aPressed = true;
            telemetry.addData("clawRotator", true);
            if (clawRotatorScoring) {
                clawRotatorScoring = false;
                clawRotator.setPosition(CLAW_ROTATOR_SCORING_POSITION);
            } else {
                clawRotatorScoring = true;
                clawRotator.setPosition((CLAW_ROTATOR_COLLECTING_POSITION));
            }
        } else if (!gamepad2.a) {
            aPressed = false;
            telemetry.addData("clawRotator", false);

        }

    }
    protected abstract AllianceColor getAllianceColor();
}
