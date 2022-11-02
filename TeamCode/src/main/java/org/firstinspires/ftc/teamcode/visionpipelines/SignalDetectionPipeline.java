package org.firstinspires.ftc.teamcode.visionpipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.Constants.BLUE_CHANNEL;
import static org.firstinspires.ftc.teamcode.Constants.GREEN_CHANNEL;
import static org.firstinspires.ftc.teamcode.Constants.RED_CHANNEL;
import static org.firstinspires.ftc.teamcode.Constants.ROI;
import static org.firstinspires.ftc.teamcode.Constants.SignalSide;
import static org.firstinspires.ftc.teamcode.Constants.YELLOW;

public class SignalDetectionPipeline extends OpenCvPipeline
{

    public static int redMean;
    public static int greenMean;
    public static int blueMean;

    private static volatile SignalSide signalSide = null;

    void inputToCb(Mat input) {
        Scalar means = Core.mean(input.submat(ROI));
        redMean = (int) means.val[RED_CHANNEL];
        greenMean = (int) means.val[GREEN_CHANNEL];
        blueMean = (int) means.val[BLUE_CHANNEL];
    }

    private void maxAverage() {
        int max = redMean;
        signalSide = SignalSide.SIDE_P;
        if (greenMean > max) {
            max = greenMean;
            signalSide = SignalSide.SIDE_R;
        }
        if (blueMean > max) {
            signalSide = SignalSide.SIDE_B;
        }
    }

    @Override
    public void init(Mat firstFrame) {
        inputToCb(firstFrame);
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToCb(input); // Process frame

        maxAverage();  // Find max color

        // Add box to camera view

        if (signalSide != null) {
            Imgproc.rectangle(input, ROI, signalSide.color, 2);
        } else {
            Imgproc.rectangle(input, ROI, YELLOW, 2);
        }

        return input;
    }

    public static SignalSide getSignaeSide()
    {
        return signalSide;
    }
}