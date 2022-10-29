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
import static org.firstinspires.ftc.teamcode.Constants.ANALOG_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.ARM_RAISER_COLLECTING_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.ARM_RAISER_SCORING_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.ARM_ROTATOR_COLLECTING_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.ARM_ROTATOR_SCORING_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.CLAW_GRABBER_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.CLAW_GRABBER_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.CLAW_ROTATOR_COLLECTING_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.CLAW_ROTATOR_SCORING_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.SPEED;
import static org.firstinspires.ftc.teamcode.Constants.TURNING_POWER_SCALAR;

public abstract class FutureTeleop extends OpMode {

    protected static AllianceColor allianceColor;

    private Gamepad g1;
    private Gamepad g2;

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
    private boolean xPressed;
    private boolean yPressed;
    private boolean aPressed;
    private boolean bPressed;

    private boolean clawGrabberOpen;
    private boolean clawRotatorScoring;
    private boolean armRotatorScoring;
    private boolean armRaiserScoring;

    private boolean manual;

    public void init() {

        g1 = new Gamepad(gamepad1);
        g2 = new Gamepad(gamepad2);

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

        armRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRaiser.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRaiser.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRaiser.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        clawGrabberOpen = false;
        clawRotatorScoring = false;
        armRotatorScoring = false;
        armRaiserScoring = false;


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

        g1.update();
        g2.update();

        getAngle();

        recalibrateGyro();

        holonomicDrive();

        switch (g2.x.getState()) {
            case PRESSED:
                manual = !manual;
                if (manual) {
                    armRotator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    armRaiser.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                } else {
                    armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armRaiser.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                break;

        }


        switch(g2.y.getState())  {
            case PRESSED:
                clawGrabber();
                break;
        }


        if (gamepad2.a && !aPressed) {
            aPressed = true;
            if (manual) {
                clawRotate();
            } else {
                clawRotate();
                armRotate();
                armRaise();
            }
        } else if (!gamepad2.a) {
            aPressed = false;
        }
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

        if (Math.abs(gamepad1LeftStickX) >= ANALOG_THRESHOLD || Math.abs(gamepad1LeftStickY) >= ANALOG_THRESHOLD) {

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

        if (gamepad1LeftTrigger >= ANALOG_THRESHOLD) {
            angleError += Math.pow(gamepad1LeftTrigger, 2);
        }
        if (gamepad1RightTrigger >= ANALOG_THRESHOLD) {
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
    private void armRaise() {
        if (manual) {
            if (Math.abs(gamepad2.right_stick_y) >= ANALOG_THRESHOLD) {
                armRaiser.setPower(gamepad2.right_stick_y);
            } else {
                armRaiser.setPower(0.0);
            }
        }

        if (armRaiserScoring) {
            armRaiserScoring = false;

            armRaiser.setTargetPosition(ARM_RAISER_SCORING_POSITION);
        } else {
            armRaiserScoring = true;
            armRaiser.setTargetPosition(ARM_RAISER_COLLECTING_POSITION);
        }
    }


    private void armRotate() {
        if (manual) {
            if (Math.abs(gamepad2.right_stick_x) >= ANALOG_THRESHOLD) {
                armRotator.setPower(gamepad2.right_stick_x);
            } else {
                armRotator.setPower(0.0);
            }
        } else {
            if (armRotatorScoring) {
                armRotatorScoring = false;
                armRotator.setTargetPosition((ARM_ROTATOR_SCORING_POSITION));
            } else {
                armRotatorScoring = true;
                armRotator.setTargetPosition((ARM_ROTATOR_COLLECTING_POSITION));
            }
        }
    }

    /**
     * Claw Controls
     */
    private void clawRotate() {
        if (manual) {
            if (clawRotatorScoring) {
                clawRotatorScoring = false;
                clawRotator.setPosition(CLAW_ROTATOR_SCORING_POSITION);
            } else {
                clawRotatorScoring = true;
                clawRotator.setPosition((CLAW_ROTATOR_COLLECTING_POSITION));
            }
        }

        if (clawRotatorScoring) {
            clawRotatorScoring = false;

            clawRotator.setPosition(CLAW_ROTATOR_SCORING_POSITION);
        } else {
            clawRotatorScoring = true;
            clawRotator.setPosition((CLAW_ROTATOR_COLLECTING_POSITION));
        }

    }

    /**
     * Toggle for claw grabber (open and close)
     */
    private void clawGrabber() {
        yPressed = true;
        if (clawGrabberOpen) {
            clawGrabberOpen = false;
            clawGrabber.setPosition(CLAW_GRABBER_CLOSE_POSITION);
        } else {
            clawGrabberOpen = true;
            clawGrabber.setPosition((CLAW_GRABBER_OPEN_POSITION));
        }

    }
    protected abstract AllianceColor getAllianceColor();
}
