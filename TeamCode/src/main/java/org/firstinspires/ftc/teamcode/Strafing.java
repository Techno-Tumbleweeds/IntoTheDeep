package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Strafing with PID", group = "Robot")
@Disabled
public class Strafing extends LinearOpMode {

    // Declare motors
    private DcMotor FrontLeft = null;
    private DcMotor FrontRight = null;
    private DcMotor BackLeft = null;
    private DcMotor BackRight = null;

    // Timer
    private ElapsedTime runtime = new ElapsedTime();

    // Constants
    static final double COUNTS_PER_MOTOR_REV = 537.7;  // Encoder counts per revolution of the motor
    static final double DRIVE_GEAR_REDUCTION = 1.0;    // Drive gear reduction (1.0 means no reduction)
    static final double WHEEL_DIAMETER_INCHES = 4.09449; // Diameter of the wheel
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415); // Counts per inch based on wheel size

    // PID coefficients
    private double kP = 0.05;  // Proportional gain
    private double kI = 0.001; // Integral gain
    private double kD = 0.01;  // Derivative gain

    // Maximum speed (you can set it based on your robot's needs)
    private double maxSpeed = 0.5;

    @Override
    public void runOpMode() {

        // Initialize motors
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "BackRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "FrontRight");

        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders
        resetEncoders();

        telemetry.addData("Status", "Ready to start");
        telemetry.update();

        waitForStart();

        pidDrive(0.2, 1, -1, -1, 1, 50, 250);
        //pidDrive(0.1, 2, -2, 10);
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

    public void pidDrive(double speed,
                         double FL_inches, double FR_inches,
                         double BL_inches, double BR_inches,
                         double timeoutS, long waitTime) {
        int FLTarget, FRTarget, BLTarget, BRTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
            // Determine new target positions
            FLTarget = FrontLeft.getCurrentPosition() + (int) (FL_inches * COUNTS_PER_INCH);
            FRTarget = FrontRight.getCurrentPosition() + (int) (FR_inches * COUNTS_PER_INCH);
            BLTarget = BackLeft.getCurrentPosition() + (int) (BL_inches * COUNTS_PER_INCH);
            BRTarget = BackRight.getCurrentPosition() + (int) (BR_inches * COUNTS_PER_INCH);

            sleep(50);

            // Set target positions
            FrontLeft.setTargetPosition(FLTarget);
            FrontRight.setTargetPosition(FRTarget);
            BackLeft.setTargetPosition(BLTarget);
            BackRight.setTargetPosition(BRTarget);

            // Turn On RUN_TO_POSITION
            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            sleep(50);

            // Set power (uses max speed)
            FrontLeft.setPower(speed);
            FrontRight.setPower(speed);
            BackLeft.setPower(speed);
            BackRight.setPower(speed);

            // Reset runtime for timeout tracking
            runtime.reset();

            // Run loop while motors are moving and within timeout
            while (opModeIsActive() && runtime.seconds() < timeoutS) {
                if (!FrontLeft.isBusy() && !FrontRight.isBusy() &&
                        !BackLeft.isBusy() && !BackRight.isBusy()) {
                    break; // Exit loop if all motors are done
                }
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

            sleep(waitTime); // Optional pause after move
        }
    }
}


         /*
    public void strafeDrive(double speed, double strafeLeft_inches, double strafeRight_inches, double timeoutS) {
        // Calculate target positions in encoder counts
        int FLTarget = FrontLeft.getCurrentPosition() + (int) (strafeLeft_inches * COUNTS_PER_INCH);
        int FRTarget = FrontRight.getCurrentPosition() + (int) (strafeRight_inches * COUNTS_PER_INCH);
        int BLTarget = BackLeft.getCurrentPosition() + (int) (strafeLeft_inches * COUNTS_PER_INCH);
        int BRTarget = BackRight.getCurrentPosition() + (int) (strafeRight_inches * COUNTS_PER_INCH);

        // Set PID variables
        double FLPrevError = 0, FRPrevError = 0, BLPrevError = 0, BRPrevError = 0;
        double FLIntegral = 0, FRIntegral = 0, BLIntegral = 0, BRIntegral = 0;
        double FLPower, FRPower, BLPower, BRPower;

        // Set motors to RUN_USING_ENCODER mode
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ElapsedTime runtime = new ElapsedTime();  // Timer to enforce timeout
        runtime.reset();

        // Run the motors using PID control until the target position is reached
        while (opModeIsActive() && runtime.seconds() < timeoutS) {
            // Calculate current errors
            double FLError = FLTarget - FrontLeft.getCurrentPosition();
            double FRError = FRTarget - FrontRight.getCurrentPosition();
            double BLError = BLTarget - BackLeft.getCurrentPosition();
            double BRError = BRTarget - BackRight.getCurrentPosition();

            // Check if all motors reached their target
            if (Math.abs(FLError) <= 10 && Math.abs(FRError) <= 10 &&
                    Math.abs(BLError) <= 10 && Math.abs(BRError) <= 10) {
                break;  // Exit loop when all motors are at target
            }

            // Calculate integral terms (limit to prevent windup)
            FLIntegral += FLError;
            FRIntegral += FRError;
            BLIntegral += BLError;
            BRIntegral += BRError;

            // Calculate derivative terms (change in error)
            double FLDerivative = FLError - FLPrevError;
            double FRDerivative = FRError - FRPrevError;
            double BLDerivative = BLError - BLPrevError;
            double BRDerivative = BRError - BRPrevError;

            // PID calculations
            FLPower = (kP * FLError) + (kI * FLIntegral) + (kD * FLDerivative);
            FRPower = (kP * FRError) + (kI * FRIntegral) + (kD * FRDerivative);
            BLPower = (kP * BLError) + (kI * BLIntegral) + (kD * BLDerivative);
            BRPower = (kP * BRError) + (kI * BRIntegral) + (kD * BRDerivative);

            // Apply calculated powers (limit to max speed range)
            FrontLeft.setPower(Math.max(-speed, Math.min(speed, FLPower)));
            FrontRight.setPower(Math.max(-speed, Math.min(speed, FRPower)));
            BackLeft.setPower(Math.max(-speed, Math.min(speed, BLPower)));
            BackRight.setPower(Math.max(-speed, Math.min(speed, BRPower)));

            // Update previous errors
            FLPrevError = FLError;
            FRPrevError = FRError;
            BLPrevError = BLError;
            BRPrevError = BRError;

            // Display telemetry for debugging
            telemetry.addData("Motor Targets", "FL: %d FR: %d BL: %d BR: %d", FLTarget, FRTarget, BLTarget, BRTarget);
            telemetry.addData("Current Positions", "FL: %d FR: %d BL: %d BR: %d",
                    FrontLeft.getCurrentPosition(), FrontRight.getCurrentPosition(),
                    BackLeft.getCurrentPosition(), BackRight.getCurrentPosition());
            telemetry.update();
        }

        // Stop motors when done
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }
    }
          */
