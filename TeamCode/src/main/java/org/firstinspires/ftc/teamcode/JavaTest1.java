package org.firstinspires.ftc.teamcode;

import android.graphics.Paint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Parallel Tests", group="Robot")
public class JavaTest1 extends LinearOpMode {
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
        claw = hardwareMap.get(Servo.class, "claw");
        clawmove = hardwareMap.get(Servo.class, "clawmove");

        // Initialize the drive system variables.
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");

        ArmJoint = hardwareMap.get(DcMotor.class, "ArmJoint");

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
