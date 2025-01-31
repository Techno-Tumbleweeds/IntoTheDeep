package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Hanging Auto", group="Robot")
@Disabled
public class HangAuto extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         FrontLeft   = null;
    private DcMotor         FrontRight  = null;
    private DcMotor ArmJoint = null;
    private DcMotor ArmMotorL = null;

    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.09449 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.3;
    static final double     TURN_SPEED              = 0.1;

    @Override
    public void runOpMode() {

        Servo claw;
        claw = hardwareMap.get(Servo.class, "claw");


        // Initialize the drive system variables.
        FrontLeft  = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "BackRight");

        ArmJoint = hardwareMap.get(DcMotor.class, "ArmJoint");
        ArmMotorL = hardwareMap.get(DcMotor.class, "ArmMotorL");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmJoint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                FrontLeft.getCurrentPosition(),
                FrontRight.getCurrentPosition());
        telemetry.update();

        ArmJoint.setTargetPosition(38);

        claw.setPosition(0.8);
        double distToPos = 0;
        double armPos = 80;
        double movepower = 0;
        double liftpower = 0;

        // Wait for the game to start (driver presses START)
        waitForStart();
        /*
        while (getRuntime() < 20) {
            opModeIsActive();
            distToPos = armPos - ArmJoint.getCurrentPosition();
            armPos = ArmJoint.getCurrentPosition() + distToPos;

            if (armPos > ArmJoint.getCurrentPosition()) {
                ArmJoint.setPower(Math.pow(1.02, 1.5 * (armPos - ArmJoint.getCurrentPosition())) - 1);
            } else {
                ArmJoint.setPower(-Math.pow(1.02, 1.2 * (ArmJoint.getCurrentPosition() - armPos)) + 1);
            }

            if (getRuntime() < 2){
                movepower = -1;
            } else if (getRuntime() > 2 && getRuntime() < 5){
                movepower = -0.2;
            }
            else if (getRuntime() > 5 && getRuntime() < 8){
                movepower = 0;
                liftpower = 0.5;
                armPos = 100;
            }
            else if (getRuntime() > 8 && getRuntime() < 10){
                movepower = 0.2;
                liftpower = 0.3;
                armPos = 120;
            }
            ArmMotorL.setPower(liftpower);
            FrontLeft.setPower(movepower);
            FrontRight.setPower(movepower);
        }

         */


        encoderDrive(DRIVE_SPEED, -10, -12, 100, 500);


        ArmJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmJoint.setPower(0.225);

        while (opModeIsActive() && ArmJoint.getCurrentPosition() <= 35) {
            // Optionally, add telemetry to monitor the current position
            telemetry.addData("Arm Position", ArmJoint.getCurrentPosition());
            telemetry.update();
        }
        //ArmJoint.setPower(-.1); // Small power to hold position
        ArmJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmJoint.setPower(0);

        // Set mode to RUN_USING_ENCODER or leave as is if no further movement is needed
        ArmJoint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        encoderDrive(DRIVE_SPEED, -12, -12, 100, 500);

        //730
        ArmMotorL.setTargetPosition(700);
        ArmMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotorL.setPower(0.2);

        while (opModeIsActive() && ArmMotorL.getCurrentPosition() <= 700) {
            // Optionally, add telemetry to monitor the current position
            telemetry.addData("Arm Position", ArmJoint.getCurrentPosition());
            telemetry.update();
        }
        ArmMotorL.setPower(0);

        encoderDrive(DRIVE_SPEED, -1, -1, 100, 500);
        claw.setPosition(0.45);

        ArmMotorL.setTargetPosition(200);
        ArmMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotorL.setPower(0.2);

        while (opModeIsActive() && ArmMotorL.getCurrentPosition() >= 200) {
            // Optionally, add telemetry to monitor the current position
            telemetry.addData("Arm Position", ArmJoint.getCurrentPosition());
            telemetry.update();
        }
        ArmMotorL.setPower(0);






        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */

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
