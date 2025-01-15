package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Auto2", group = "Sensor")
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
    static final double WHEEL_DIAMETER_INCHES = 4.09449; // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_SPEED = 0.6;


    double timeoutS = 5000;

    private void stopMotors() {
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }

    public void strafeDrive (double strafeDistance, double speed, long wait){

        int newLeftFrontTarget = 0, newRightFrontTarget = 0, newLeftBackTarget = 0, newRightBackTarget = 0;

        if (opModeIsActive()) {

            newLeftFrontTarget = FrontLeft.getCurrentPosition() - (int) (strafeDistance * COUNTS_PER_INCH);
            newRightFrontTarget = FrontRight.getCurrentPosition() + (int) (strafeDistance * COUNTS_PER_INCH);
            newLeftBackTarget = BackLeft.getCurrentPosition() + (int) (strafeDistance * COUNTS_PER_INCH);
            newRightBackTarget = BackRight.getCurrentPosition() - (int) (strafeDistance * COUNTS_PER_INCH);

            FrontLeft.setTargetPosition(newLeftFrontTarget);
            FrontRight.setTargetPosition(newRightFrontTarget);
            BackLeft.setTargetPosition(-newLeftBackTarget);
            BackRight.setTargetPosition(-newRightBackTarget);

            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            FrontLeft.setPower(speed);
            FrontRight.setPower(speed);
            BackLeft.setPower(-speed);
            BackRight.setPower(-speed);


            while (opModeIsActive() && runtime.seconds() < timeoutS && FrontLeft.isBusy() && FrontRight.isBusy()) {
                //telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                //telemetry.addData("Currently at", " %7d :%7d", FrontLeft.getCurrentPosition(), FrontRight.getCurrentPosition());
                //telemetry.update();
            }


            sleep(wait);
            stopMotors();
        }
    }

    public void encoderDrive (int leftInches, int rightInches, double speed, long wait) {

        int newLeftFrontTarget = 0, newRightFrontTarget = 0, newLeftBackTarget = 0, newRightBackTarget = 0;

        if (opModeIsActive()) {

            newLeftFrontTarget = FrontLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = FrontRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = BackLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightBackTarget = BackRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

                FrontLeft.setTargetPosition(newLeftFrontTarget);
                FrontRight.setTargetPosition(newRightFrontTarget);
                BackLeft.setTargetPosition(newLeftBackTarget);
                BackRight.setTargetPosition(newRightBackTarget);

            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            FrontLeft.setPower(speed);
            FrontRight.setPower(speed);
            BackLeft.setPower(speed);
            BackRight.setPower(speed);


            while (opModeIsActive() && runtime.seconds() < timeoutS && FrontLeft.isBusy() && FrontRight.isBusy()) {
                //telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightFrontTarget);
                //telemetry.addData("Currently at", " %7d :%7d", FrontLeft.getCurrentPosition(), FrontRight.getCurrentPosition());
                //telemetry.update();
            }


            sleep(wait);
            stopMotors();
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

        encoderDrive(-50, -50, 0.5, 250);

        sleep(1500);

        strafeDrive(5,0.5, 250);

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
