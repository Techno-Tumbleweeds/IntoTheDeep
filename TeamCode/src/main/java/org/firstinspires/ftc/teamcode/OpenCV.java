package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    // === MOTOR DECLARATIONS ===
    DcMotor leftFront, rightFront, leftBack, rightBack;

    private ElapsedTime timer = new ElapsedTime();
    private enum State { IDLE, WAITING, TURNING }
    private State currentState = State.IDLE;

    boolean goingLeft = true;

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

        // === INITIALIZE MOTORS ===
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Set direction based on drivetrain config
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        telemetry.addLine("Camera is streaming...");
        telemetry.addData("Frame count", Webcam1.getFrameCount());

        if (pipeline != null) {
            String pos = pipeline.position;
            telemetry.addData("Detected Position", pos);

            switch (currentState) {
                case IDLE:
                    if (pos.equals("LEFT")) {
                        strafeLeft();
                    } else if (pos.equals("RIGHT")) {
                        strafeRight();
                    } else if (pos.equals("CENTER")) {
                        stopDrive();
                    } else if (pos.equals("NONE")) {
                        if (timer.seconds() > 1.0){
                            goingLeft = !goingLeft;
                            timer.reset();
                        }
                        if (goingLeft){
                            strafeLeft();
                        } else {
                            strafeRight();
                        }
                    }
                    break;
            }
        } else {
            telemetry.addLine("Pipeline not initialized.");
        }

        telemetry.update();
    }


    // === MOTOR CONTROL METHODS ===
    public void strafeLeft() {
        leftFront.setPower(-0.15);
        leftBack.setPower(0.15);
        rightFront.setPower(0.15);
        rightBack.setPower(-0.15);
    }

    public void strafeRight() {
        leftFront.setPower(0.15);
        leftBack.setPower(-0.15);
        rightFront.setPower(-0.15);
        rightBack.setPower(0.15);
    }

    public void stopDrive() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);


    }
    public void sensedTurn(){
        leftFront.setPower(0.3);
        leftBack.setPower(0.3);
        rightFront.setPower(-0.3);
        rightBack.setPower(-0.3);
    }
    class ExamplePipeline extends OpenCvPipeline {
        public String position = "NONE";

        @Override
        public Mat processFrame(Mat input) {
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            Scalar lowerYellow = new Scalar(20, 100, 100);
            Scalar upperYellow = new Scalar(30, 255, 255);
            Mat mask = new Mat();
            Core.inRange(hsv, lowerYellow, upperYellow, mask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);


            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            double maxArea = 0;
            Rect bestRect = null;
            int minX = Integer.MAX_VALUE;
            int minY = Integer.MAX_VALUE;
            int maxX = 0;
            int maxY = 0;
            boolean found = false;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > 500) {
                    Rect rect = Imgproc.boundingRect(contour);
                    minX = Math.min(minX, rect.x);
                    minY = Math.min(minY, rect.y);
                    maxX = Math.max(maxX, rect.x + rect.width);
                    maxY = Math.max(maxY, rect.y + rect.height);
                    found = true;
                }
            }

            Rect mergedRect = null;
            if (found) {
                mergedRect = new Rect(minX, minY, maxX - minX, maxY - minY);
            }

            if (mergedRect != null) {
                Imgproc.rectangle(input, mergedRect, new Scalar(0, 255, 0), 2);
                int centerX = mergedRect.x + mergedRect.width / 2;

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