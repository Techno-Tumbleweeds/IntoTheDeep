package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Auto2", group = "Sensor")
@Disabled
public class AutoWithStrafing extends LinearOpMode {

    private final int READ_PERIOD = 1; // Read period in seconds
    private HuskyLens Cam;

    // Declare motors
    private DcMotor FrontLeft = null;
    private DcMotor FrontRight = null;
    private DcMotor BackLeft = null;
    private DcMotor BackRight = null;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.09449;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_SPEED = 0.6;


    double timeoutS = 15000;

    private void stopMotors() {
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }

    public void strafeDrive(double strafeDistance, double speed, long wait) {
        if (opModeIsActive()) {
            // Calculate encoder targets for strafing
            int newLeftFrontTarget = FrontLeft.getCurrentPosition() - (int) (strafeDistance * COUNTS_PER_INCH);
            int newRightFrontTarget = FrontRight.getCurrentPosition() + (int) (strafeDistance * COUNTS_PER_INCH);
            int newLeftBackTarget = BackLeft.getCurrentPosition() + (int) (strafeDistance * COUNTS_PER_INCH);
            int newRightBackTarget = BackRight.getCurrentPosition() - (int) (strafeDistance * COUNTS_PER_INCH);

            // Set targets
            FrontLeft.setTargetPosition(newLeftFrontTarget);
            FrontRight.setTargetPosition(newRightFrontTarget);
            BackLeft.setTargetPosition(newLeftBackTarget);
            BackRight.setTargetPosition(newRightBackTarget);

            // Set motors to RUN_TO_POSITION
            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            // Apply speed to the motors for strafing
            FrontLeft.setPower(speed);
            FrontRight.setPower(speed);
            BackLeft.setPower(speed);
            BackRight.setPower(speed);

            // Wait for the motion to complete
            while (opModeIsActive() && runtime.seconds() < timeoutS &&
                    FrontLeft.isBusy() && FrontRight.isBusy() &&
                    BackLeft.isBusy() && BackRight.isBusy()) {
                telemetry.addData("Target", "LF: %d, RF: %d, LB: %d, RB: %d",
                        newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Current", "LF: %d, RF: %d, LB: %d, RB: %d",
                        FrontLeft.getCurrentPosition(), FrontRight.getCurrentPosition(),
                        BackLeft.getCurrentPosition(), BackRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop motors and wait
            stopMotors();
            sleep(wait);
        }
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


    @Override
    public void runOpMode() {
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");

        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Cam = hardwareMap.get(HuskyLens.class, "Cam");
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

        telemetry.addData(">", "Press Start to begin");
        telemetry.update();
        waitForStart();

        encoderDrive(0.25, -50, -50, 5000, 5);

        sleep(1500);

        strafeDrive(5, .15, 1000);


        //strafeDrive(5,0.5, 250);

/*
        while (opModeIsActive()) {
            if (!rateLimit.hasExpired()) continue;
            rateLimit.reset();

            HuskyLens.Block[] blocks = Cam.blocks();
            telemetry.addData("Block count", blocks.length);

            if (blocks.length > 0) {
                HuskyLens.Block block = blocks[0];
                if (block.height < 75) {
                    FrontLeft.setPower(0.2);
                    FrontRight.setPower(0.2);
                } else if (block.height > 79) {
                    FrontLeft.setPower(0.2);
                    FrontRight.setPower(0.2);
                } else {
                    stopMotors();
                    telemetry.addData("Object aligned", "True");
                }
            } else {
                stopMotors();

            telemetry.update();


        }
        } */

    }
}
