package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Specimen Auto", group="Robot")
public class WingSpecimensAuto extends LinearOpMode {
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

        LiftKit.setTargetPosition(500);
        ArmJoint.setTargetPosition(600);
        LiftKit.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ArmJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FrontLeft.setTargetPosition(0);
        FrontRight.setTargetPosition(0);
        BackLeft.setTargetPosition(0);
        BackRight.setTargetPosition(0);


        double liftPos = 500;
        double distToPosLift = liftPos - LiftKit.getCurrentPosition();

        clawmove.setPosition(0.48);

        //initializes claw
        claw.setPosition(1);
        sleep(250);
        claw.setPosition(0.6);
        sleep(1200);
        claw.setPosition(1);

        //auto begins here =========================================================
        waitForStart();

        //moves Arm
        ArmJoint.setTargetPosition(860);
        ArmJoint.setPower(0.6);

        //drives toward submersible
        drive(0.35, 1250, -1, -1, -1, -1, 5);

        //moves arm higher
        ArmJoint.setTargetPosition(1150);
        ArmJoint.setPower(0.6);

        sleep(1250);

        //drives away from submersible
        drive(0.35, 200, 1, 1, 1, 1, 5);

        claw.setPosition(0.6);
        sleep(150);

        drive(0.35, 250, 1, 1, 1, 1, 5);

        //strafes left
        drive(0.35, 1300, -1, 1, 1, -1, 5);

        //drives toward submersible
        drive(0.35, 1300, -1, -1, -1, -1, 5);

        drive(0.35, 300, -1, 1, 1, -1, 5);

        drive(0.35, 1700, 1, 1, 1, 1, 5);

        drive(0.35, 1600, -1, -1, -1, -1, 5);

        drive(0.35, 325, -1, 1, 1, -1, 5);

        drive(0.35, 1700, 1, 1, 1, 1, 5);

        ArmJoint.setTargetPosition(3750);
        ArmJoint.setPower(0.3);

        LiftKit.setTargetPosition(1000);
        LiftKit.setPower(-0.6);

        sleep(750);

        drive(0.35, 750, -1, -1, -1, -1, 5);

        sleep(950);

        drive(0.35, 450, 1, 1, 1, 1, 5);
        drive(0.2, 250, 1, 1, 1, 1, 5);

        sleep(100);

        //LiftKit.setTargetPosition(950);
        //LiftKit.setPower(0.4);

        claw.setPosition(1);

        sleep(3000);
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
