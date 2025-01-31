package org.firstinspires.ftc.teamcode;

import android.graphics.Paint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Right Auto", group="Robot")
public class AutoWing extends LinearOpMode {
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;

    DcMotor ArmJoint;
    Servo claw;
    Servo clawmove;


    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {


        // Initialize the drive system variables.
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");

        ArmJoint = hardwareMap.get(DcMotor.class, "ArmJoint");

        claw = hardwareMap.get(Servo.class, "claw");
        clawmove = hardwareMap.get(Servo.class, "clawmove");

        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ArmJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FrontLeft.setTargetPosition(0);
        FrontRight.setTargetPosition(0);
        BackLeft.setTargetPosition(0);
        BackRight.setTargetPosition(0);

        ArmJoint.setTargetPosition(0);


        waitForStart();

        //preloaded sample

        //strafe off the wall
        drive(0.25, 200, -1, 1, 1, -1, 5);
        //push the sample to wing and backs out
        drive(0.4, 1300, 1, 1, 1, 1, 5);
        drive(0.4, 250, -1, -1, -1, -1, 5);

        drive(0.425, 1000, 1, -1, 1, -1, 15);
        drive(0.45, 300, 1, 1, 1, 1, 5);
        drive(0.4, 2100, -1, -1, -1, -1, 20);

        drive(0.25, 450, -1, 1, 1, -1, 5);
        drive(0.4, 1650, 1, 1, 1, 1, 5);
        drive(0.4, 1650, -1, -1, -1, -1, 5);
        drive(0.25, 435, -1, 1, 1, -1, 5);
        drive(0.4, 1500, 1, 1, 1, 1, 5);

        drive(0.4, 400, -1, -1, -1, -1, 5);

        sleep(5000);

        claw.setPosition(0.25);
        clawmove.setPosition(0.48);

        ArmJoint.setTargetPosition(3400);
        ArmJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        ArmJoint.setPower(0.4);

        while (opModeIsActive() &&
                (ArmJoint.isBusy()) && runtime.seconds() < 5) {

            // Display it for the driver
            //telemetry.addData("Running to", " %7d :%7d", newLeftFTarget, newRightFTarget);
            telemetry.addData("Currently at", ArmJoint.getCurrentPosition());
            telemetry.update();
        }
        drive(0.4, 175, 1, 1, 1, 1, 5);


        ArmJoint.setPower(0);


        //drive(0.4, 1500, -1, -1, -1, -1, 5);

        /*
        //turns facing the wall and squares

        //first sample

        //strafes to line up with first sample
        drive(0.2, 400, 1, -1, -1, 1, 5);
        //turns to better push sample in
        drive(0.2, 96, 1, 0, 1, 0, 5);
        //pushes sample into wing and backs out
        drive(0.4, 1775, 1, 1, 1, 1, 5);
        drive(0.4, 1790, -1, -1, -1, -1, 5);

        //2nd sample

        //strafes to 2nd sample
        drive(0.2, 650, 1, -1, -1, 1, 5);
        //squares to 2nd sample sample
        drive(0.2, 85, 0, 1, 0, 1, 5);

        drive(0.4, 1450, 1, 1, 1, 1, 5);

        drive(0.45, 1050, -1, -1, -1, -1, 5);

        drive(0.4, 950, 1, -1, 1, -1, 15);

        drive(0.6, 525, -1, -1, -1, -1, 5);

        ArmJoint.setTargetPosition(3000);
        ArmJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        ArmJoint.setPower(0.35);

        while (opModeIsActive() &&
                (ArmJoint.isBusy()) && runtime.seconds() < 5) {

            // Display it for the driver
            //telemetry.addData("Running to", " %7d :%7d", newLeftFTarget, newRightFTarget);
            telemetry.addData("Currently at", ArmJoint.getCurrentPosition());
            telemetry.update();
        }

        ArmJoint.setPower(0);

         */

    }
    public void drive(double speed, int targetPos,
                      int flDirection, int frDirection,
                      int blDirection, int brDirection,
                      double Timeout) {

        FrontLeft.setTargetPosition(FrontLeft.getTargetPosition() + (targetPos * flDirection));
        FrontRight.setTargetPosition(FrontRight.getTargetPosition() + (targetPos * frDirection));
        BackLeft.setTargetPosition(BackLeft.getTargetPosition() + (targetPos * blDirection));
        BackRight.setTargetPosition(BackRight.getTargetPosition() + (targetPos * brDirection));

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        FrontLeft.setPower(speed * flDirection);
        FrontRight.setPower(speed * frDirection);
        BackLeft.setPower(speed * blDirection);
        BackRight.setPower(speed * brDirection);

        while (opModeIsActive() &&
                (runtime.seconds() < Timeout) &&
                (FrontLeft.isBusy() && FrontRight.isBusy() &&
                        BackLeft.isBusy() && BackRight.isBusy())) {

            // Display it for the driver
            //telemetry.addData("Running to", " %7d :%7d", newLeftFTarget, newRightFTarget);
            telemetry.addData("Currently at", " at %7d :%7d",
                    FrontLeft.getCurrentPosition(), FrontRight.getCurrentPosition());
            telemetry.update();
        }

        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);

        sleep(475);
    }
}
