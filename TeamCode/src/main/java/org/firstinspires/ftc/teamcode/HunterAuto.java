package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoHunter", group = "Auto")
public class HunterAuto extends LinearOpMode {

    // Declare hardware components
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;
    private DcMotor ArmMotorR;
    private DcMotor ArmMotorL;
    private DcMotor ArmJoint;
    private Servo claw;

    @Override
    public void runOpMode() {
        // Initialize hardware components
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        ArmMotorR = hardwareMap.get(DcMotor.class, "ArmMotorR");
        ArmMotorL = hardwareMap.get(DcMotor.class, "ArmMotorL");
        //ArmJoint = hardwareMap.get(Servo.class, "ArmJoint");
        claw = hardwareMap.get(Servo.class, "claw");

        // Set motor directions
        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        // Set arm motor behavior
        ArmMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        ArmMotorL.setDirection(DcMotorSimple.Direction.REVERSE);

        // Enable braking behavior
        ArmMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders
        ArmMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        BackLeft.setPower(0.5);
        BackRight.setPower(0.5);

        sleep(5000);

        BackRight.setPower(0);
        BackLeft.setPower(0);


    }
}
