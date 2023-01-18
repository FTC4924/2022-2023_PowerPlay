package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.visionpipelines.SignalDetectionPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class Camera extends SubsystemBase {
    private OpenCvWebcam webcam;
    private SignalDetectionPipeline cameraPipeline;
    private Constants.SignalSide signalSide;

    public Camera() {
        super();
    }
}
