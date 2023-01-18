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

public class SignalDetectionPipeline extends OpenCvPipeline {

    private int redMean;
    private int greenMean;
    private int blueMean;
    private int loops;

    private volatile SignalSide signalSide = SignalSide.SIDE_R;

    void inputToCb(Mat input) {
        Scalar means = Core.mean(input.submat(ROI));
        redMean += (int) means.val[RED_CHANNEL];
        greenMean += (int) means.val[GREEN_CHANNEL];
        blueMean += (int) means.val[BLUE_CHANNEL];

        loops++;

        redMean /= loops;
        greenMean /= loops;
        blueMean /= loops;
    }

    private void maxAverage() {
        int max = redMean;
        signalSide = SignalSide.SIDE_R;
        if (blueMean > max) {
            max = blueMean;
            signalSide = SignalSide.SIDE_B;
        }
        if (greenMean > max) {
            signalSide = SignalSide.SIDE_P;
        }
    }

    @Override
    public void init(Mat firstFrame) {
        inputToCb(firstFrame);

        maxAverage();
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

    public void clearSignalSide() {
        redMean = 0;
        greenMean = 0;
        blueMean = 0;
        loops = 0;
    }

    public SignalSide getSignalSide()
    {
        return signalSide;
    }
}