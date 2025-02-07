package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Sample Auto", group="Robot")
public class SamplesAuto extends LinearOpMode {
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;

    DcMotor ArmJoint;

    DcMotor LiftKit;

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

        LiftKit = hardwareMap.get(DcMotor.class, "ArmMotorL");



        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        //LiftKit.setDirection(DcMotor.Direction.REVERSE);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ArmJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftKit.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LiftKit.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LiftKit.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FrontLeft.setTargetPosition(0);
        FrontRight.setTargetPosition(0);
        BackLeft.setTargetPosition(0);
        BackRight.setTargetPosition(0);

        ArmJoint.setTargetPosition(2275);

        double liftPos = 1350;
        double distToPosLift = liftPos - LiftKit.getCurrentPosition();

        clawmove.setPosition(0.48);

        //intializes claw
        claw.setPosition(1);
        sleep(250);
        claw.setPosition(0.6);        sleep(1200);
        claw.setPosition(1);

        waitForStart();
        //start =======================================================================================================================
        //strafes off wall
        drive(0.35, 100, 1, -1, -1, 1, 5);

        ArmJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmJoint.setPower(0.6);

        while (distToPosLift > 50 && opModeIsActive()){
            distToPosLift = liftPos - LiftKit.getCurrentPosition();
            liftPos = LiftKit.getCurrentPosition() + distToPosLift;

            LiftKit.setPower(-(liftPos - LiftKit.getCurrentPosition())/200);
            telemetry.addData("ArmPosition: ", LiftKit.getCurrentPosition());
            telemetry.addData("LiftPosition: ", liftPos);
            telemetry.addData("Arm Power:",(liftPos - LiftKit.getCurrentPosition())/200);
            telemetry.update();
        }
        LiftKit.setPower(-0.2);

        /*
        while (ArmJoint.isBusy() && opModeIsActive()){
            telemetry.addData("ArmPosition: ", ArmJoint.getCurrentPosition());
            telemetry.addData("Arm Power:", ArmJoint.getPower());
            telemetry.update();
        }*/



        //moves lift kit
        //LiftKit.setTargetPosition(1300);  ============ first lift
        //LiftKit.setPower(-0.6);

        sleep(200);
        //
        drive(0.35, 300, 1, 1, 1, 1, 5);

        drive(0.35, 250, -1, 1, -1, 1, 5);
        sleep(500);
        claw.setPosition(0.6);
        sleep(300);

        drive(0.35, 225, -1, -1, -1, -1, 5);


        drive(0.35, 1200, 1, -1, 1, -1, 5);

        ArmJoint.setTargetPosition(3750);
        ArmJoint.setPower(0.3);

        //backs up into wall
        drive(0.3, 400, -1, -1, -1, -1, 5);

        drive(0.35, 785, 1, 1, 1, 1, 5);

        sleep(300);
        claw.setPosition(0.6);
        drive(0.35, 330, -1, 1, 1, -1, 5);

        /*LiftKit.setTargetPosition(175);
        LiftKit.setPower(0.6);
        sleep(2000);
        claw.setPosition(1);
        sleep(200);

        ArmJoint.setTargetPosition(2330);
        LiftKit.setTargetPosition(1700);
        LiftKit.setPower(-0.6);
        sleep(1000);

         */
/*
        LiftKit.setTargetPosition(1200);
        LiftKit.setPower(0.6);

 */
        liftPos = 150;
        distToPosLift = liftPos - LiftKit.getCurrentPosition();

        while (distToPosLift < -50 && opModeIsActive()){
            distToPosLift = liftPos - LiftKit.getCurrentPosition();
            liftPos = LiftKit.getCurrentPosition() + distToPosLift;

            LiftKit.setPower(-(liftPos - LiftKit.getCurrentPosition())/200);
            telemetry.addData("ArmPosition: ", LiftKit.getCurrentPosition());
            telemetry.addData("LiftPosition: ", liftPos);
            telemetry.addData("Arm Power:",(liftPos - LiftKit.getCurrentPosition())/200);
            telemetry.update();
        }
        LiftKit.setPower(-0.15);

        sleep(400);
        //closes on second sample
        claw.setPosition(1);

        sleep(600);
/*
        LiftKit.setTargetPosition(1300);
        LiftKit.setPower(-0.6);

 */
        ArmJoint.setTargetPosition(2330);
        ArmJoint.setPower(-0.5);

        liftPos = 1350;
        distToPosLift = liftPos - LiftKit.getCurrentPosition();
        while (distToPosLift > 50 && opModeIsActive()){
            distToPosLift = liftPos - LiftKit.getCurrentPosition();
            liftPos = LiftKit.getCurrentPosition() + distToPosLift;

            LiftKit.setPower(-(liftPos - LiftKit.getCurrentPosition())/200);
            telemetry.addData("ArmPosition: ", LiftKit.getCurrentPosition());
            telemetry.addData("LiftPosition: ", liftPos);
            telemetry.addData("Arm Power:",(liftPos - LiftKit.getCurrentPosition())/200);
            telemetry.update();
        }
        LiftKit.setPower(-0.15);

        drive(0.35, 1500, -1, 1, -1, 1, 5);

        drive(0.35, 400, 1, 1, 1, 1, 5);
        sleep(700);
        //drops second sample
        claw.setPosition(0.6);
        sleep(680);

        drive(0.35, 1200, -1, -1, -1, -1, 5);
        clawmove.setPosition(0.15);
        drive(0.35, 600, 1, -1, 1, -1, 5);
        ArmJoint.setTargetPosition(3700);
        sleep(200);
        //strafe right before picking up last sample
        drive(0.3, 180, 1, -1, -1, 1, 5);

        drive(0.35, 128, 1, 1, 1, 1, 5);
/*172
        LiftKit.setTargetPosition(0);
        LiftKit.setPower(0.6);
 */
        liftPos = 125;
        distToPosLift = liftPos - LiftKit.getCurrentPosition();

        while (distToPosLift < -50 && opModeIsActive()){
            distToPosLift = liftPos - LiftKit.getCurrentPosition();
            liftPos = LiftKit.getCurrentPosition() + distToPosLift;

            LiftKit.setPower(-(liftPos - LiftKit.getCurrentPosition())/200);
            telemetry.addData("ArmPosition: ", LiftKit.getCurrentPosition());
            telemetry.addData("LiftPosition: ", liftPos);
            telemetry.addData("Arm Power:",(liftPos - LiftKit.getCurrentPosition())/200);
            telemetry.update();
        }
        LiftKit.setPower(-0.15);

        sleep(750);
        claw.setPosition(1);

        //lifts kit after grabbing 3rd sample
        sleep(750);
/*
        LiftKit.setTargetPosition(1300);
        LiftKit.setPower(-0.6);
 */
        ArmJoint.setTargetPosition(2330);
        ArmJoint.setPower(-0.5);

        liftPos = 1350;
        distToPosLift = liftPos - LiftKit.getCurrentPosition();
        while (distToPosLift > 50 && opModeIsActive()){
            distToPosLift = liftPos - LiftKit.getCurrentPosition();
            liftPos = LiftKit.getCurrentPosition() + distToPosLift;

            LiftKit.setPower(-(liftPos - LiftKit.getCurrentPosition())/200);
            telemetry.addData("ArmPosition: ", LiftKit.getCurrentPosition());
            telemetry.addData("LiftPosition: ", liftPos);
            telemetry.addData("Arm Power:",(liftPos - LiftKit.getCurrentPosition())/200);
            telemetry.update();
        }
        LiftKit.setPower(-0.15);


        clawmove.setPosition(0.48);

        drive(0.35, 650, -1, 1, -1, 1, 5);

        drive(0.35, 1050, 1, 1, 1, 1, 5);
        //drops last sample
        sleep(1000);
        claw.setPosition(0.6);        sleep(500);

        //backs up after last sample
        drive(0.5, 1050, -1, -1, -1, -1, 5);
/*
        LiftKit.setTargetPosition(0);
        LiftKit.setPower(0.6);
 */
        liftPos = 1350;
        distToPosLift = liftPos - LiftKit.getCurrentPosition();
        while (distToPosLift < -50 && opModeIsActive()){
            distToPosLift = liftPos - LiftKit.getCurrentPosition();
            liftPos = LiftKit.getCurrentPosition() + distToPosLift;

            LiftKit.setPower(-(liftPos - LiftKit.getCurrentPosition())/200);
            telemetry.addData("ArmPosition: ", LiftKit.getCurrentPosition());
            telemetry.addData("LiftPosition: ", liftPos);
            telemetry.addData("Arm Power:",(liftPos - LiftKit.getCurrentPosition())/200);
            telemetry.update();
        }
        LiftKit.setPower(-0.15);

        ArmJoint.setTargetPosition(2500);
        ArmJoint.setPower(0.3);







        /*
        liftPos = 300;

        while (distToPosLift > 50 && opModeIsActive()){
            distToPosLift = liftPos - LiftKit.getCurrentPosition();
            liftPos = LiftKit.getCurrentPosition() + distToPosLift;

            LiftKit.setPower(-(liftPos - LiftKit.getCurrentPosition())/200);
            telemetry.addData("ArmPosition: ", LiftKit.getCurrentPosition());
            telemetry.addData("LiftPosition: ", liftPos);
            telemetry.addData("Arm Power:",(liftPos - LiftKit.getCurrentPosition())/200);
            telemetry.update();
        }

        drive(0.5, 425, -1, -1, -1, -1, 5);

         */

        sleep(2000);
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

        sleep(200); //was 800
    }
}
