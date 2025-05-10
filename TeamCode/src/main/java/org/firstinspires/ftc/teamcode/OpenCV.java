package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class OpenCV extends OpMode {

    OpenCvWebcam Webcam1 = null;

    @Override
    public void init() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam1");

        int cameraMonitorViewId = hardwareMap.appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        Webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        Webcam1.setPipeline(new ExamplePipeline());

        Webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                Webcam1.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
            }
        });
    }

    @Override
    public void loop() {
        telemetry.addLine("Camera is streaming...");
        telemetry.addData("Frame count", Webcam1.getFrameCount());
        telemetry.update();
    }

    class ExamplePipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Just return the raw camera feed for now
            return input;
        }
    }
}
