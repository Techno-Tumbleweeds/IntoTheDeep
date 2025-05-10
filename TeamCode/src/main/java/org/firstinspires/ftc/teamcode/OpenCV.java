package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class OpenCV extends OpMode {

    OpenCvWebcam Webcam1 = null;
    ExamplePipeline pipeline;

    @Override
    public void init() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam1");

        int cameraMonitorViewId = hardwareMap.appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        Webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        pipeline = new ExamplePipeline();  // store pipeline reference
        Webcam1.setPipeline(pipeline);

        Webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                Webcam1.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });
    }

    @Override
    public void loop() {
        telemetry.addLine("Camera is streaming...");
        telemetry.addData("Frame count", Webcam1.getFrameCount());

        if (pipeline != null) {
            telemetry.addData("Detected Position", pipeline.position);
        } else {
            telemetry.addLine("Pipeline not initialized.");
        }

        telemetry.update();
    }

    class ExamplePipeline extends OpenCvPipeline {
        public String position = "NONE";  // make position accessible

        @Override
        public Mat processFrame(Mat input) {
            // Convert RGB to HSV
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Create yellow mask
            Scalar lowerYellow = new Scalar(20, 100, 100);
            Scalar upperYellow = new Scalar(30, 255, 255);
            Mat mask = new Mat();
            Core.inRange(hsv, lowerYellow, upperYellow, mask);

            // Clean up the mask
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

            // Find contours
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find largest valid contour
            double maxArea = 0;
            Rect bestRect = null;
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > 500) {
                    Rect rect = Imgproc.boundingRect(contour);
                    float aspectRatio = (float) rect.width / rect.height;

                    // Accept more rectangular shapes (either wide or tall)
                    if ((aspectRatio > 1.5 || aspectRatio < 0.66) && area > maxArea) {
                        maxArea = area;
                        bestRect = rect;
                    }
                }
            }

            // Determine position
            if (bestRect != null) {
                Imgproc.rectangle(input, bestRect, new Scalar(0, 255, 0), 2);
                int centerX = bestRect.x + bestRect.width / 2;

                if (centerX < input.cols() / 3) {
                    position = "LEFT";
                } else if (centerX < 2 * input.cols() / 3) {
                    position = "CENTER";
                } else {
                    position = "RIGHT";
                }
            } else {
                position = "NONE";
            }

            return input;
        }
    }
}