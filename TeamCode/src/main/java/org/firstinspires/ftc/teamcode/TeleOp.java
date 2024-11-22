package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;




@TeleOp(name = "TeleOp")
public class TeleOp extends OpMode {




    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;
    DcMotor ArmMotor;
    Servo ArmJoint;
    Servo claw;




    double armPos =1;




    double motorSpeed = 0.8;








    boolean isMoving = false;
    //double armPosfreeze = 0.35;




    @Override
    public void init() {
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class,"BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");




        claw = hardwareMap.get(Servo.class, "claw");
        ArmJoint = hardwareMap.get(Servo.class, "ArmJoint"); // Initialize the Servo




        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.REVERSE);




        ArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);




        //Slows drop of lift kit
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);








        telemetry.addData("Initialization:", "Initialized");
        telemetry.update();
    }




    @Override
    public void loop() {
        FrontRight.setPower((((gamepad1.right_trigger - gamepad1.left_trigger - gamepad1.left_stick_x)) - (gamepad1.right_stick_x * 0.85)) * motorSpeed);
        FrontLeft.setPower((((gamepad1.right_trigger - gamepad1.left_trigger + gamepad1.left_stick_x)) + (gamepad1.right_stick_x * 0.85)) * motorSpeed);
        BackLeft.setPower((((gamepad1.right_trigger - gamepad1.left_trigger + gamepad1.left_stick_x)) - (gamepad1.right_stick_x)) * motorSpeed);
        BackRight.setPower((((gamepad1.right_trigger - gamepad1.left_trigger - gamepad1.left_stick_x)) + (gamepad1.right_stick_x)) * motorSpeed);








        //top basket
        if (gamepad2.dpad_up) {
            //sets position for arm height an moves it there
            //ArmMotor.setTargetPosition(2750);
            //ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //ArmMotor.setPower(0.7);




            armPos = 0.99;
        }




        //top rung
        if (gamepad2.dpad_left) {
            //sets position for arm height an moves it there
            //ArmMotor.setTargetPosition(1200);
            //ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //ArmMotor.setPower(0.6);




        }




        //bottom
        if (gamepad2.dpad_down) {
            //sets position for arm height an moves it there
            //ArmMotor.setTargetPosition(275);
            //ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //ArmMotor.setPower(0.25);


            armPos = 0;
        }




      /*
      if (gamepad2.triangle) {
          armPos = 0.65;
          armPosfreeze = armPos;
      } else if (gamepad2.square) {
          armPos = 0.45;
          armPosfreeze = armPos;
      } else if (gamepad2.cross) {
          armPos = 0.36;
          armPosfreeze = armPos;
      } else if (gamepad2.left_bumper) {
          armPos = 0.31;
      } else if (!gamepad2.left_bumper) {
          armPos = armPosfreeze;
      }




       */




        if(gamepad1.triangle){
            motorSpeed = 0.8;
        } else if (gamepad1.square) {
            motorSpeed = 0.5;
        } else if (gamepad1.cross) {
            motorSpeed = 0.275;
        }




        //manual control
        if (!ArmMotor.isBusy()) {
            //Stop both motors when target position is reached
            ArmMotor.setPower(-gamepad2.right_stick_y);
            // Set motors back to RUN_USING_ENCODER mode for other operations
            ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }








        ArmJoint.setPosition(armPos + (0 - gamepad2.left_stick_y)




        );




        if (gamepad2.right_bumper){
            claw.setPosition(0.8);
        }
        else{
            claw.setPosition(0.5);
        }




        // Example telemetry for servo position
        telemetry.addData("Servo Position", ArmJoint.getPosition());
        telemetry.addData("Trigger value: ", gamepad2.left_stick_y / 2 + armPos);
        telemetry.update();
    }
}

