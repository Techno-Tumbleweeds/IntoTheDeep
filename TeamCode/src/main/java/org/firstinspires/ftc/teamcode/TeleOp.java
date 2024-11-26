package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends OpMode {
    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;
    DcMotor ArmMotorR;
    DcMotor ArmMotorL;
    Servo claw;

    double motorSpeed = 0.8;
    //boolean isMoving = false;
    //double armPosfreeze = 0.35;

    @Override
    public void init() {
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class,"BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        ArmMotorR = hardwareMap.get(DcMotor.class, "ArmMotorR");
        ArmMotorL = hardwareMap.get(DcMotor.class, "ArmMotorL");

        claw = hardwareMap.get(Servo.class, "claw");

        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.REVERSE);


        ArmMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ArmMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Initialization:", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        BackLeft.setPower((((gamepad1.right_trigger - gamepad1.left_trigger + gamepad1.left_stick_x)) - (gamepad1.right_stick_x)) * motorSpeed);
     /*

        if(gamepad1.triangle){
        } else if (gamepad1.square) {
        } else if (gamepad1.cross) {
        }


        }


        if (!ArmMotorL.isBusy()) {
            //Stop both motors when target position is reached
            ArmMotorL.setPower(gamepad2.right_stick_y);
            // Set motors back to RUN_USING_ENCODER mode for other operations
            ArmMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }






        // Example telemetry for servo position
        telemetry.update();
    }
}