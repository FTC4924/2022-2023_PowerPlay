package org.firstinspires.ftc.teamcode;

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

import androidx.annotation.FloatRange;

/**
 * Contains constants for all of the programs in one file for easy access.
 */

public class Constants {

    public enum CONTROLLER_ELEMENT_STATE {
        IDLE,
        PRESSED,
        HELD,
        RELEASED
    }

    public enum TELEOP_STATE {
        MANUAL,
        AUTO,
        TEST
    }

    public enum SignalSide {
        SIDE_R(RED),
        SIDE_B(BLUE),
        SIDE_P(GREEN);

        public final Scalar color;

        private SignalSide(Scalar color) {
            this.color = color;
        }
    }

    public enum WristState {
        COLLECT(0.8253),
        SCORE(0.1508);

        public final double pos;

        WristState(double pos) {
            this.pos = pos;
        }
    }

    public enum GripperState {
        CLOSE(0.38),
        OPEN(0.8);

        public final double pos;

        GripperState(double pos) {
            this.pos = pos;
        }
    }

    public enum LiftPos {
        LIFT_UP(2350),
        LIFT_DOWN(20);

        public final int pos;

        LiftPos(int pos) {
            this.pos = pos;
        }
    }

    public enum ArmPos {
        ARM_SCORING(2250),
        ARM_COLLECTING(-2250);

        public final int pos;

        ArmPos(int pos) {
            this.pos = pos;
        }
    }

    @FloatRange(from=0.0, to=1.0)
    protected static final double ANGLE_ERROR_TOLERANCE = 0.05;

    /**
     * Number of motor encoder ticks per foot.
     */
    public static final double TICKS_PER_FOOT = 434.76;

    /**
     * Defines the dead zone for controller input.
     */
    @FloatRange(from=0.0, to=1.0)
    protected static final double ANALOG_THRESHOLD = 0.05;
    /**
     * Defines the tolerance for the angle error.
     */
    @FloatRange(from=0.0, to=1.0)
    public static final double HEADING_ERROR_TOLERANCE = 0.05;
    /**
     * How close the encoder needs to get to the target position for autonomous to move to the next command.
     */
    public static final double ENCODER_POSITION_TOLERANCE = 20.0;

    public static final double TURNING_ENCODER_POSITION_SCALAR = 10.0;
    public static final double TURNING_POWER_SCALAR = 0.35;
    public static final double CHASSIS_RADIUS = 0.77;

    public static final double HOLONOMIC_SPEED = 0.75;

    /*
    All the constants below are part of image processing.
     */
    public static final Scalar RED = new Scalar(255, 0, 0);
    public static final Scalar GREEN = new Scalar(0, 255, 0);
    public static final Scalar BLUE = new Scalar(0, 0, 255);
    public static final Scalar YELLOW = new Scalar(255, 255, 0);
    public static final Rect ROI = new Rect(new Point(550,500), new Point(800,650));
    public static final int RED_CHANNEL = 0;
    public static final int GREEN_CHANNEL = 1;
    public static final int BLUE_CHANNEL = 2;
    public static final int RESOLUTION_WIDTH = 1280;
    public static final int RESOLUTION_HEIGHT = 960;
    public static final String WEBCAM_RECORDING_FILE = "/Movies/match_recording.mp4";

    public static final double CLAW_GRABBER_OPEN_POSITION = 0.6640;
    public static final double CLAW_GRABBER_CLOSE_POSITION =  0.0781;
    public static final double CLAW_ROTATOR_SCORING_POSITION = 0.1406;
    public static final double CLAW_ROTATOR_COLLECTING_POSITION = 0.8267;
    public static final double ARM_POWER = 0.75;
    public static final int ARM_ROTATOR_SCORING_POSITION = 2250;
    public static final int ARM_ROTATOR_COLLECTING_POSITION = -2250;
    public static final int ARM_RAISER_SCORING_POSITION = 2350;
    public static final int ARM_RAISER_COLLECTING_POSITION = 20;
    public static final int ARM_RAISER_MAX_POSITION = 2400;
    public static final int ARM_RAISER_MIN_POSITION = -20;
    public static final double ARM_RAISER_RAISE_SPEED = 0.5;
    public static final double ARM_RAISER_LOWER_SPEED = 0.25;
    public static final double ARM_CORRECTIVE_POWER = 0.5;

}