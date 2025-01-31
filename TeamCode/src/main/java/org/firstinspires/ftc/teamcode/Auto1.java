package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto1 Current", group="Robot")
public class Auto1 extends LinearOpMode {
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {


        // Initialize the drive system variables.
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");

        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FrontLeft.setTargetPosition(0);
        FrontRight.setTargetPosition(0);
        BackLeft.setTargetPosition(0);
        BackRight.setTargetPosition(0);

        waitForStart();

        //preloaded sample

        //strafe off the wall
        drive(0.2, 200, 1, -1, -1, 1, 5);
        //push the sample to wing and backs out
        drive(0.4, 880, 1, 1, 1, 1, 5);
        drive(0.4, 880, -1, -1, -1, -1, 5);
        //turns facing the wall and squares
        drive(0.4, 1000, -1, 1, -1, 1, 15);
        drive(0.3, 165, 1, 1, 1, 1, 5);
        //backs up
        drive(0.3, 2100, -1, -1, -1, -1, 20);

        //first sample

        //strafes to line up with first sample
        drive(0.2, 400, 1, -1, -1, 1, 5);
        //turns to better push sample in
        drive(0.2, 94, 1, 0, 1, 0, 5);
        //pushes sample into wing and backs out
        drive(0.4, 1775, 1, 1, 1, 1, 5);
        drive(0.4, 1790, -1, -1, -1, -1, 5);

        //2nd sample

        //strafes to 2nd sample
        drive(0.2, 575, 1, -1, -1, 1, 5);
        //squares to 2nd sample sample
        drive(0.2, 85, 0, 1, 0, 1, 5);

        drive(0.4, 1450, 1, 1, 1, 1, 5);

        drive(0.4, 1200, -1, -1, -1, -1, 5);

        drive(0.4, 950, 1, -1, 1, -1, 15);

        drive(0.4, 325, -1, -1, -1, -1, 5);

        //drive(0.4, 2000, -1, -1, -1, -1, 5);

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
