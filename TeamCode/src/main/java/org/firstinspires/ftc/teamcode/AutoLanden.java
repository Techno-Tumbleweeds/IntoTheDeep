package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/*
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Auto")

public class AutoAuto extends LinearOpMode {
    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;
    DcMotor ArmMotorR;
    DcMotor ArmMotorL;
    Servo ArmJoint;
    Servo claw;

    @Override
    public void runOpMode(){

        //Finds all motors in robot configuration
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class,"BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        ArmMotorR = hardwareMap.get(DcMotor.class, "ArmMotorR");
        ArmMotorL = hardwareMap.get(DcMotor.class, "ArmMotorL");

        //finds all servos
        claw = hardwareMap.get(Servo.class, "claw");

        //sets direction of the wheel motors
        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        //sets direction of arm motors
        ArmMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        ArmMotorL.setDirection(DcMotorSimple.Direction.REVERSE);

        //resets encoders in motors
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double COUNTS_PER_INCH = 5;

        //sets them to run using encoders
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Front starting at",  "%7d :%7d",
                FrontLeft.getCurrentPosition(),
                FrontRight.getCurrentPosition());
        telemetry.update();

        waitForStart();
/*
        public void encoderDrive(double speed,
        double leftInches, double rightInches) {
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
                FrontLeft.setPower(Math.abs(speed));
                FrontRight.setPower(Math.abs(speed));

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() && (FrontLeft.isBusy() && FrontRight.isBusy())) {
                }
                    // Display it for the driver.
                    telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                    telemetry.addData("Currently at",  " at %7d :%7d",
                            FrontLeft.getCurrentPosition(), FrontRight.getCurrentPosition());
                    telemetry.update();
                }

                encoderDrive(0.8, 10, 10);

                // Stop all motion;
                FrontLeft.setPower(0);
                FrontRight.setPower(0);

                // Turn off RUN_TO_POSITION
                FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                sleep(250);   // optional pause after each move.
            }


        }
    }
    */
