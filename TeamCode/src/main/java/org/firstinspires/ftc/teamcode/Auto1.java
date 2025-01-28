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

    private ElapsedTime runtime = new ElapsedTime();

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


        waitForStart();

        drive(0.1, 900, 1, 1, 1, 1, 10);
        sleep(20000);
    }
    public void drive(double speed, int targetPos,
                      double flDirection, double frDirection,
                      double blDirection, double brDirection,
                      double Timeout) {
        FrontLeft.setTargetPosition(targetPos);
        FrontRight.setTargetPosition(targetPos);
        BackLeft.setTargetPosition(targetPos);
        BackRight.setTargetPosition(targetPos);

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FrontLeft.setPower(flDirection*speed);
        FrontRight.setPower(frDirection*speed);
        BackLeft.setPower(blDirection*speed);
        BackRight.setPower(brDirection*speed);

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
    }
}
