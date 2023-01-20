package org.firstinspires.ftc.teamcode;

import android.os.Build;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.visionpipelines.SignalDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import androidx.annotation.RequiresApi;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.teamcode.Constants.ANALOG_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.ARM_CORRECTIVE_POWER;
import static org.firstinspires.ftc.teamcode.Constants.ARM_POWER;
import static org.firstinspires.ftc.teamcode.Constants.ARM_RAISER_MAX_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.ARM_RAISER_MIN_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.ArmPos;
import static org.firstinspires.ftc.teamcode.Constants.ArmPos.ARM_COLLECTING;
import static org.firstinspires.ftc.teamcode.Constants.ArmPos.ARM_SCORING;
import static org.firstinspires.ftc.teamcode.Constants.CONTROLLER_ELEMENT_STATE;
import static org.firstinspires.ftc.teamcode.Constants.CONTROLLER_ELEMENT_STATE.HELD;
import static org.firstinspires.ftc.teamcode.Constants.CONTROLLER_ELEMENT_STATE.PRESSED;
import static org.firstinspires.ftc.teamcode.Constants.CONTROLLER_ELEMENT_STATE.RELEASED;
import static org.firstinspires.ftc.teamcode.Constants.GripperState;
import static org.firstinspires.ftc.teamcode.Constants.GripperState.CLOSE;
import static org.firstinspires.ftc.teamcode.Constants.HOLONOMIC_SPEED;
import static org.firstinspires.ftc.teamcode.Constants.LiftPos;
import static org.firstinspires.ftc.teamcode.Constants.LiftPos.LIFT_DOWN;
import static org.firstinspires.ftc.teamcode.Constants.LiftPos.LIFT_UP;
import static org.firstinspires.ftc.teamcode.Constants.RESOLUTION_HEIGHT;
import static org.firstinspires.ftc.teamcode.Constants.RESOLUTION_WIDTH;
import static org.firstinspires.ftc.teamcode.Constants.TELEOP_STATE;
import static org.firstinspires.ftc.teamcode.Constants.TELEOP_STATE.AUTO;
import static org.firstinspires.ftc.teamcode.Constants.TELEOP_STATE.MANUAL;
import static org.firstinspires.ftc.teamcode.Constants.TELEOP_STATE.TEST;
import static org.firstinspires.ftc.teamcode.Constants.TURNING_POWER_SCALAR;
import static org.firstinspires.ftc.teamcode.Constants.WristState;
import static org.firstinspires.ftc.teamcode.Constants.WristState.COLLECT;

public abstract class TeleopBase extends OpMode {

    protected static AllianceColor allianceColor;

    private OpenCvWebcam webcam;
    private int x1, x2, y1, y2;  // Webcam area of interest tuning

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

    private double clawRotatorPosition;
    private double clawGrabberPosition;

    // Gyro sensor
    private BNO055IMU imu;

    private Orientation angles;
    private double angleOffset;
    private double currentRobotAngle;
    private double targetAngle;

    private DigitalChannel liftLimit;
    private DigitalChannel armLimit;
    private BinaryComponent liftLimitComponent;
    private BinaryComponent armLimitComponent;

    private int liftOffset;
    private int armOffset;

    private GripperState gripperState;
    private WristState wristState;
    private ArmPos armPos;
    private LiftPos liftPos;

    private TELEOP_STATE state;

    public void init() {

        g1 = new Gamepad(gamepad1);
        g2 = new Gamepad(gamepad2);
        state = MANUAL;

        allianceColor = getAllianceColor();

        gripperState = CLOSE;
        wristState = COLLECT;
        armPos = ARM_SCORING;
        liftPos = LIFT_UP;

        /*Instantiating the motor and servo objects as their appropriate motor/servo in the
        configuration on the robot*/
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        armRotator = hardwareMap.get(DcMotor.class, "arm");
        armRaiser = hardwareMap.get(DcMotor.class, "lift");

        clawRotator = hardwareMap.get(Servo.class, "wrist");
        clawRotator.setPosition(wristState.pos);
        clawGrabber = hardwareMap.get(Servo.class, "gripper");
        clawGrabber.setPosition(gripperState.pos);

        armRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotator.setTargetPosition(0);
        armRotator.setDirection(DcMotorSimple.Direction.REVERSE);
        armRaiser.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRaiser.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRaiser.setTargetPosition(0);

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

        liftLimit = hardwareMap.get(DigitalChannel.class, "liftLimit");
        liftLimit.setMode(DigitalChannel.Mode.INPUT);
        //armLimit = hardwareMap.get(DigitalChannel.class, "armLimit");
        //armLimit.setMode(DigitalChannel.Mode.INPUT);

        liftLimitComponent = new BinaryComponent();
        armLimitComponent = new BinaryComponent();

        configureState();
    }

    /*@Override
    public void init_loop() {
        g2.update();

        if (g2.left_stick_x.getState() == HELD) {
            x1 += (int) (g2.left_stick_x.component);
        }
        if (g2.left_stick_y.getState() == HELD) {
            y1 += (int) (g2.left_stick_y.component);
        }
        if (g2.right_stick_x.getState() == HELD) {
            x2 += (int) (g2.right_stick_x.component);
        }
        if (g2.right_stick_y.getState() == HELD) {
            y2 += (int) (g2.right_stick_y.component);
        }

        SignalDetectionPipeline.ROI = new Rect(new Point(x1,y1), new Point(x2,y2));

        telemetry.addData("ROI x1", x1);
        telemetry.addData("ROI y1", y1);
        telemetry.addData("ROI x2", x2);
        telemetry.addData("ROI y2", y2);
    }*/

    public void start() {
        resetRuntime();
    }

    @RequiresApi(api = Build.VERSION_CODES.O)
    public void loop() {
        telemetry.addData("MANUAL/AUTO: ", state);
        telemetry.addData("g2x", g2.x.getState());
        telemetry.addData("g2_left_stick_y", g2.left_stick_y.component);
        telemetry.addData("g2_left_stick_x", g2.left_stick_x.component);
        telemetry.addData("armRotator.position", armRotator.getCurrentPosition());
        telemetry.addData("armRaiser.position", armRaiser.getCurrentPosition());
        telemetry.addData("clawGrabber.position", clawGrabber.getPosition());
        telemetry.addData("clawRotator.position", clawRotator.getPosition());
        telemetry.addData("wristState", wristState.name());
        telemetry.addData("wristState.pos", wristState.pos);

        // Update the custom gamepad objects.
        g1.update();
        g2.update();
        liftLimitComponent.update(liftLimit.getState());
        //armLimitComponent.update(armLimit.getState());

        telemetry.addData("liftLimit", liftLimitComponent.getState());

        // Update the internal IMU orientation variables to eliminate extra calls to the IMU in loop.
        getAngle();

        // Zero the gyro.
        switch (g1.b.getState()) {
            case PRESSED:
                recalibrateGyro();
                break;
        }

        // Update all driving information.
        holonomicDrive();

        // Switch the active mode. Disabled in TEST mode.
        switch (g2.x.getState()) {
            case PRESSED:
                switch (state) {
                    case MANUAL:  // Switch to Auto
                        state = AUTO;
                        break;
                    case AUTO:  // Switch to Manual
                        state = MANUAL;
                        break;

                    /*case MANUAL:
                        break;*/
                    case TEST:
                        break;
                }
                configureState();
        }

        // Run the loop method for the current mode.
        switch (state) {
            case MANUAL:
                manualLoop();
                break;
            case AUTO:
                autoLoop();
                break;
            case TEST:
                testLoop();
                break;
        }
    }

    private void manualLoop() {
        /*if (liftLimitComponent.getState() == PRESSED || liftLimitComponent.getState() == RELEASED) {
            liftOffset = armRaiser.getCurrentPosition();
        }*/

        // Actuate the arm raiser.
        switch (g2.left_stick_y.getState()) {
            case HELD:
                /*if (g2.right_stick_button.getState() != HELD) {   // If the right stick button is held, let the arm go past the limit.
                    if ((g2.left_stick_y.component > 0 && armRaiser.getCurrentPosition() < ARM_RAISER_MAX_POSITION) ||  // If the arm is trying to go up and can do so, or
                            (g2.left_stick_y.component < 0 && armRaiser.getCurrentPosition() > ARM_RAISER_MIN_POSITION)) { // if the arm is trying to go down and can do so.
                        armRaiser.setPower(g2.left_stick_y.component);

                    } else {
                        armRaiser.setPower(0.0);
                    }
                }

                else armRaiser.setPower(g2.left_stick_y.component);  // Lets the arm move without limits if the right stick is pressed.
*/
                if (g2.right_stick_button.getState() != HELD) {
                    if (armRaiser.getMode() != DcMotor.RunMode.RUN_TO_POSITION) armRaiser.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if (g2.left_stick_y.component > 0) armRaiser.setTargetPosition(ARM_RAISER_MAX_POSITION - armOffset);
                    else if (g2.left_stick_y.component < 0 /*&& liftLimit.getState()*/) armRaiser.setTargetPosition(ARM_RAISER_MIN_POSITION - armOffset);
                    else /*if (!liftLimit.getState())*/ armRaiser.setTargetPosition(-armOffset);

                    armRaiser.setPower(g2.left_stick_y.component);
                } else {
                    if (armRaiser.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) armRaiser.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    armRaiser.setPower(g2.left_stick_y.component);
                }
                break;
            case RELEASED:
                armRaiser.setTargetPosition(armRaiser.getCurrentPosition());
                armRaiser.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armRaiser.setPower(ARM_CORRECTIVE_POWER);
                break;
        }

        // Rotate the arm.
        switch (g2.right_stick_y.getState()) {
            case PRESSED:
                armRotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            case HELD:
                armRotator.setPower(g2.right_stick_y.component * ARM_POWER);
                break;
            case RELEASED:
                armRotator.setTargetPosition(armRotator.getCurrentPosition());
                armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armRotator.setPower(ARM_CORRECTIVE_POWER);
                break;
        }

        // Actuate the claw grabber.
        if (g2.y.getState() == PRESSED || g2.left_bumper.getState() == PRESSED) {
            clawGrabber();
        }

        // Rotate the claw
        if (g2.b.getState() == PRESSED || g2.right_bumper.getState() == PRESSED) {
            clawRotate();
        }

        // Reset the arm raiser position.
        if (g2.dpad_up.getState() == CONTROLLER_ELEMENT_STATE.RELEASED && g2.right_trigger.component > 0.5) {
            DcMotor.RunMode mode = armRaiser.getMode();
            armRaiser.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armRaiser.setTargetPosition(0);
            armRaiser.setMode(mode);
        }
    }

    /**
     * Loop with more automation, such as
     */
    private void autoLoop() {
        telemetry.addData("motormode", armRaiser.getMode());
        telemetry.addData("armRaiserTargetPos", armRaiser.getTargetPosition());

        // Rotate the arm, raise the arm, and rotate the claw
        switch (g2.b.getState()) {
            case PRESSED:
                armRotate();
                armRaise();
                clawRotate();
                break;
        }

        // Actuate the claw grabber.
        switch (g2.y.getState()) {
            case PRESSED:
                clawGrabber();
                break;
        }
    }

    /**
     * Alternative loop for testing.
     */
    private void testLoop() {
        // Manually rotate the claw.
        switch (g2.right_stick_x.getState()) {
            case PRESSED:
            case HELD:
                clawRotatorPosition += g2.right_stick_x.component / 100;
                break;
        }
        clawRotator.setPosition(clawRotatorPosition);

        // Manually actuate the claw grabber.
        switch (g2.left_stick_x.getState()) {
            case PRESSED:
            case HELD:
                clawGrabberPosition += g2.left_stick_x.component / 100;
                break;
        }
        clawGrabber.setPosition(clawGrabberPosition);
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
        angleOffset = angles.firstAngle;
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
            double power = Math.sqrt(Math.pow(gamepad1LeftStickX, 2) + Math.pow(gamepad1LeftStickY, 2)) * HOLONOMIC_SPEED;

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
        switch (liftPos) {
            case LIFT_DOWN:
                liftPos = LIFT_UP;
                break;
            case LIFT_UP:
                liftPos = LIFT_DOWN;
                break;
        }
        armRaiser.setTargetPosition(liftPos.pos);
    }


    private void armRotate() {
        switch (armPos) {
            case ARM_COLLECTING:
                armPos = ARM_SCORING;
                break;
            case ARM_SCORING:
                armPos = ARM_COLLECTING;
                break;
        }
        armRotator.setTargetPosition(armPos.pos);
    }

    /**
     * Claw Controls
     */
    private void clawRotate()   {
        switch (wristState) {
            case SCORE:
                wristState = WristState.COLLECT;
                break;
            case COLLECT:
                wristState = WristState.SCORE;
                break;
        }
        clawRotator.setPosition(wristState.pos);
    }

    /**
     * Toggle for claw grabber (open and close)
     */
    private void clawGrabber() {
        switch (gripperState) {
            case CLOSE:
                gripperState = GripperState.OPEN;
                break;
            case OPEN:
                gripperState = GripperState.CLOSE;
                break;
        }
        clawGrabber.setPosition(gripperState.pos);
    }

    private void configureState() {
        switch (state) {
            case AUTO:  // Switch to Auto
                armRotator.setPower(ARM_CORRECTIVE_POWER);
                armRaiser.setPower(ARM_CORRECTIVE_POWER);
                armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armRaiser.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case MANUAL:  // Switch to Manual
                armRotator.setPower(0.0);
                armRaiser.setPower(0.0);
                armRotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armRaiser.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;

                    /*case MANUAL:
                        break;*/
            case TEST:
                int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

                webcam.setPipeline(new SignalDetectionPipeline());
                webcam.setMillisecondsPermissionTimeout(2500);
                webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        webcam.startStreaming(RESOLUTION_WIDTH, RESOLUTION_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                        telemetry.addData("Webcam", "Setup Finished");
                    }

                    public void onError(int errorCode) {
                        telemetry.speak("The web cam wasn't initialised correctly! Error code: " + errorCode);
                        telemetry.addData("Webcam", "Setup Failed! Error code: " + errorCode);
                    }
                });
                break;
        }
    }

    protected abstract AllianceColor getAllianceColor();
}
