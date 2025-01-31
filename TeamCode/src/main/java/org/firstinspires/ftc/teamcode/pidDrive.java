package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "PID with Encoders", group = "Robot")
@Disabled
public class pidDrive extends LinearOpMode {

    // Declare motors
    private DcMotor FrontLeft, FrontRight, BackLeft, BackRight;

    // PID constants
    private double kP = 0.1;   // Proportional constant
    private double kI = 0.001; // Integral constant
    private double kD = 0.01;  // Derivative constant

    // Constants for encoder conversion
    static final double COUNTS_PER_MOTOR_REV = 537.7;  // Encoder counts per revolution of the motor
    static final double DRIVE_GEAR_REDUCTION = 1.0;    // Gear reduction factor
    static final double WHEEL_DIAMETER_INCHES = 4.0;   // Diameter of the wheel
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415); // Encoder counts per inch of travel

    // Declare variables for the PID loop
    private double previousError = 0;
    private double totalError = 0;

    private double distanceInches = 0;

    @Override
    public void runOpMode() {
        // Initialize motors
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");

        // Set motor directions
        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        // Reset and set encoders
        resetEncoders();

        telemetry.addData("Status", "Ready to start");
        telemetry.update();

        waitForStart();

        // Example of driving forward 24 inches
        pidDrive(24);
    }

    // Reset motor encoders
    private void resetEncoders() {
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    // PID-based movement function
    public void pidDrive(double FL_Inches) {
        // Calculate target encoder counts
        int targetPosition = (int) (distanceInches * COUNTS_PER_INCH);

        // Set target positions for each motor
        FrontLeft.setTargetPosition(FrontLeft.getCurrentPosition() + targetPosition);
        FrontRight.setTargetPosition(FrontRight.getCurrentPosition() + targetPosition);
        BackLeft.setTargetPosition(BackLeft.getCurrentPosition() + targetPosition);
        BackRight.setTargetPosition(BackRight.getCurrentPosition() + targetPosition);

        // Set motors to run to the target positions
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start motors at initial power (can be adjusted based on your needs)
        FrontLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackLeft.setPower(0.5);
        BackRight.setPower(0.5);

        // Run the PID loop until the robot reaches the target position
        while (opModeIsActive() && (
                FrontLeft.isBusy() && FrontRight.isBusy() &&
                        BackLeft.isBusy() && BackRight.isBusy())) {

            // Calculate error (difference between current position and target)
            double error = targetPosition - FrontLeft.getCurrentPosition();

            // Proportional term (based on error)
            double pTerm = kP * error;

            // Integral term (sum of errors over time)
            totalError += error;
            double iTerm = kI * totalError;

            // Derivative term (rate of change of error)
            double dTerm = kD * (error - previousError);

            // Compute PID output
            double output = pTerm + iTerm + dTerm;

            // Set motor power (adjust as needed)
            FrontLeft.setPower(output);
            FrontRight.setPower(output);
            BackLeft.setPower(output);
            BackRight.setPower(output);

            // Update previous error
            previousError = error;

            telemetry.addData("PID Output", output);
            telemetry.update();
        }

        // Stop motors once the target position is reached
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }
}
