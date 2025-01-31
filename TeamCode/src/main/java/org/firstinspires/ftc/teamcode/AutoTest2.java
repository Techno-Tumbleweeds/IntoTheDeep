package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoTest2", group="Robot")
@Disabled
public class AutoTest2 extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor FrontLeft = null;
    private DcMotor FrontRight = null;
    private DcMotor BackLeft = null;
    private DcMotor BackRight = null;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.09449;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_SPEED = 0.3;
    static final double TURN_SPEED = 0.1;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        //FrontRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        //BackRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");

        // Set motor directions
        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.FORWARD);

        // Set zero power behavior
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
/*
        // Reset and set encoders
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


 */
        // Send telemetry message to indicate successful Encoder reset
        /*
        telemetry.addData("Starting at", "%7d :%7d",
                FrontLeft.getCurrentPosition(),
                FrontRight.getCurrentPosition());
        telemetry.update();

         */

        // Wait for the game to start (driver presses START)
        waitForStart();

        FrontLeft.setTargetPosition(1000);
        FrontRight.setTargetPosition(1000);
        BackLeft.setTargetPosition(1000);
        BackRight.setTargetPosition(1000);

        // Turn On RUN_TO_POSITION
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Reset the timeout time and start motion
        FrontLeft.setPower(-0.35);
        FrontRight.setPower(0.35);
        BackLeft.setPower(0.35);
        BackRight.setPower(-0.35);

        //encoderDrive(0.35, 5, 5, 5, 5, 5, 500);

        // Autonomous actions go here

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display final telemetry message.
    }
/*
    public void encoderDrive(double speed,
                             double LeftFront, double RightFront,
                             double LeftBack, double RightBack,
                             double Timeout, long Wait) {
        int newLeftFTarget;
        int newRightFTarget;
        int newLeftBTarget;
        int newRightBTarget;

        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftFTarget = FrontLeft.getCurrentPosition() + (int)(LeftFront * COUNTS_PER_INCH);
            newRightFTarget = FrontRight.getCurrentPosition() + (int)(RightFront * COUNTS_PER_INCH);
            newLeftBTarget = BackLeft.getCurrentPosition() + (int)(LeftBack * COUNTS_PER_INCH);
            newRightBTarget = BackRight.getCurrentPosition() + (int)(RightBack * COUNTS_PER_INCH);

            /*
            FrontLeft.setTargetPosition(newLeftFTarget);
            FrontRight.setTargetPosition(newRightFTarget);
            BackLeft.setTargetPosition(newLeftBTarget);
            BackRight.setTargetPosition(newRightBTarget);


            FrontLeft.setTargetPosition(-1000);
            FrontRight.setTargetPosition(1000);
            BackLeft.setTargetPosition(1000);
            BackRight.setTargetPosition(-1000);

            // Turn On RUN_TO_POSITION
            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Reset the timeout time and start motion
            runtime.reset();
            FrontLeft.setPower(speed);
            FrontRight.setPower(speed);
            BackLeft.setPower(speed);
            BackRight.setPower(speed);

            // Keep looping while motors are running and time hasn't run out
            while (opModeIsActive() &&
                    (runtime.seconds() < Timeout) &&
                    (FrontLeft.isBusy() && FrontRight.isBusy() &&
                            BackLeft.isBusy() && BackRight.isBusy())) {

                // Display it for the driver
                telemetry.addData("Running to", " %7d :%7d", newLeftFTarget, newRightFTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        FrontLeft.getCurrentPosition(), FrontRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(Wait);  // Optional pause after each move
        }

    }
    */
}
