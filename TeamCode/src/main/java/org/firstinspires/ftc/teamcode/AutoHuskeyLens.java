package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Auto HL", group = "Sensor")
public class AutoHuskeyLens extends LinearOpMode {

    private final int READ_PERIOD = 1;
    private HuskyLens Cam;

    // Declare motors
    private DcMotor         FrontLeft   = null;
    private DcMotor         FrontRight  = null;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.09449;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    public void encoderDrive(double speed,
                                double leftInches, double rightInches,
                                double timeoutS, long waitTime,
                                double motorSpeed) {
        int newLeftTarget;
        int newRightTarget;


        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = FrontLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = FrontRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            FrontLeft.setTargetPosition(newLeftTarget);
            FrontRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            FrontLeft.setPower(Math.abs(speed) * motorSpeed);
            FrontRight.setPower(Math.abs(speed) * motorSpeed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (FrontLeft.isBusy() && FrontRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        FrontLeft.getCurrentPosition(), FrontRight.getCurrentPosition());
                telemetry.update();

            }

            sleep(waitTime);   // optional pause after each move.

        }
    }

    private void stopMotors() {
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
    }

    private void moveForward(double power) {
        FrontLeft.setPower(power);
        FrontRight.setPower(power);
    }

    @Override
    public void runOpMode() {

        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");

        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                FrontLeft.getCurrentPosition(),
                FrontRight.getCurrentPosition());
        telemetry.update();

        Cam = hardwareMap.get(HuskyLens.class, "Cam");
        long time = System.currentTimeMillis();
        int searchDirection = 1;

        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        if (!Cam.knock()) {
            telemetry.addData(">>", "Problem communicating with " + Cam.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        Cam.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);
        telemetry.update();
        waitForStart();

        encoderDrive(DRIVE_SPEED,  -80,  -80, 10, 500, 0.85);  // S1: Forward 47 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   -37, 37, 25, 500);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(TURN_SPEED,   10, -10, 25, 325, 0.5);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(TURN_SPEED,   16, 16, 25, 325, 0.2);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(TURN_SPEED,   4, 22, 25, 750, 0.45);  // S2: Turn Right 12 Inches with 4 Sec timeout

        while (opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            HuskyLens.Block[] blocks = Cam.blocks();
            telemetry.addData("Block count", blocks.length);


            for (HuskyLens.Block block : blocks) {
                telemetry.addData("Block", block.toString());


                // Example: Move forward if object with ID 1 is detected
                if (block.id == 1) {
                    if (block.height < 75){

                        FrontLeft.setPower(0.1);
                        FrontRight.setPower(0.1);

                        while (block.height < 75) {
                            // You can also add a small delay to avoid running the loop too fast
                            try {
                                Thread.sleep(100); // Pause for 100 ms
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                        }

                        // Once the block height is 75 or more, stop the motors
                        FrontLeft.setPower(0);
                        FrontRight.setPower(0);
                    } else if (block.height > 79) {

                        FrontLeft.setPower(-0.1);
                        FrontRight.setPower(-0.1);

                        try {
                            Thread.sleep(100); // Pause for 100 ms
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }else {
                        FrontLeft.setPower(0);
                        FrontRight.setPower(0);

                    }

                    moveForward(0.5); // Move forward at 50% power
                } else {
                    telemetry.addData("JIGGLE",searchDirection);
                    telemetry.update();
                    FrontLeft.setPower(0.1 * searchDirection);
                    FrontRight.setPower(0.1 * searchDirection);
                    if (System.currentTimeMillis() - time > 2500) {
                        time = System.currentTimeMillis();
                        searchDirection *= -1;
                    }
                    System.currentTimeMillis();
                }
            }


            telemetry.update();
        }
    }
}
