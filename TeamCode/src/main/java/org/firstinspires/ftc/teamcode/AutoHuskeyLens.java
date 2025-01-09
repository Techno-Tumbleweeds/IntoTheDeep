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
    private DcMotor FrontLeft;
    private DcMotor FrontRight;

    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.09449 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                FrontLeft.getCurrentPosition(),
                FrontRight.getCurrentPosition());
        telemetry.update();

        Cam = hardwareMap.get(HuskyLens.class, "Cam");
        FrontLeft = hardwareMap.get(DcMotor.class, "left_motor");
        FrontRight = hardwareMap.get(DcMotor.class, "right_motor");

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

        while (opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            HuskyLens.Block[] blocks = Cam.blocks();
            telemetry.addData("Block count", blocks.length);

            /*
            for (HuskyLens.Block block : blocks) {
                telemetry.addData("Block", block.toString());


                // Example: Move forward if object with ID 1 is detected
                if (block.id == 1) {
                    moveForward(0.5); // Move forward at 50% power
                } else {
                    stopMotors();
                }
            }

             */


            telemetry.update();
        }

        public void encoderDrive(double speed,
                double leftInches, double rightInches,
                double timeoutS, long waitTime) {
            int newLeftTarget;
            int newRightTarget;


            // Ensure that the OpMode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newLeftTarget = FrontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                newRightTarget = FrontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
                FrontLeft.setTargetPosition(newLeftTarget);
                FrontRight.setTargetPosition(newRightTarget);

                // Turn On RUN_TO_POSITION
                FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                FrontLeft.setPower(Math.abs(speed));
                FrontRight.setPower(Math.abs(speed));

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
                    telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                    telemetry.addData("Currently at",  " at %7d :%7d",
                            FrontLeft.getCurrentPosition(), FrontRight.getCurrentPosition());
                    telemetry.update();

                }

                // Stop all motion;
                FrontLeft.setPower(0);
                FrontRight.setPower(0);

                // Turn off RUN_TO_POSITION
                FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                sleep(waitTime);   // optional pause after each move.
            }
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
    }
