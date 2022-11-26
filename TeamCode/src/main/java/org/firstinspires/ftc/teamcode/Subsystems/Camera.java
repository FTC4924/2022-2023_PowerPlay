package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants.SignalSide;
import org.firstinspires.ftc.teamcode.ResourceManager;
import org.firstinspires.ftc.teamcode.visionpipelines.SignalDetectionPipeline;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import androidx.annotation.NonNull;

/*
 * Created by Brendan Clark on 02/27/2022 at 10:35 AM.
 */

/**
 * Subsystem for controlling the camera.
 *
 * @see Subsystem
 */
public class Camera extends Subsystem {
    private final OpenCvWebcam webcam;

    private SignalDetectionPipeline pipeline;
    private volatile boolean streaming;


    /**
     * Initialize the camera and OpenCV pipeline.
     *
     * @see HardwareMap
     * @see Telemetry
     */
    public Camera(@NonNull ResourceManager resourceManager, String name, HardwareMap hardwareMap) {
        super(resourceManager, name);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(resourceManager.removeDevice(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        if (webcam == null) telemetry.addData(name, webcam);

        pipeline = new SignalDetectionPipeline();
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500);     /////// Diagnosing mystery error that has no origin, but is in the below commented out section.
        /*webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {     ////// FOUND    "Webcam 1" is registered in the resource manager as "Sonix Technology Co., Ltd. USB 2.0 Camera". Likely source of above issue.
            @Override
            public void onOpened() {
                webcam.startStreaming(RESOLUTION_WIDTH, RESOLUTION_HEIGHT, OpenCvCameraRotation.SIDEWAYS_RIGHT);
                telemetry.addData(name, "Setup Finished");
                streaming = true;
            }

            public void onError(int errorCode) {
                telemetry.speak("The web cam wasn't initialised correctly! Error code: " + errorCode);
                telemetry.addData(name, "Setup Failed! Error code: " + errorCode);
                streaming = true;
            }
        });*/

        //while (!streaming) {}  // Wait until the webcam has finished opening and is streaming.

    }

    @Override
    public void loop() {
        if (streaming) telemetry.addData(name, getSignal().toString());
    }

    @Override
    public void stop() {
        stopStreaming();
    }

    public SignalSide getSignal() {
        return pipeline.getSignalSide();
    }
    /**
     * Stops streaming images from the camera (and, by extension, stops invoking your vision pipeline),
     * without closing ({@linkplain OpenCvWebcam#closeCameraDevice()}) the connection to the camera.
     *
     * @see OpenCvWebcam#stopStreaming()
     */
    public void stopStreaming() {
        //if (streaming) webcam.stopStreaming();
    }
}
