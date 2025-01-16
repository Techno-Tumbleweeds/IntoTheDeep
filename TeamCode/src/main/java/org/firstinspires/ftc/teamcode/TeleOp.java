package org.firstinspires.ftc.teamcode;

//imports packages
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends OpMode {
    //Declares names of components
    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;
    DcMotor ArmMotorR;
    DcMotor ArmMotorL;
    DcMotor ArmJoint;
    Servo claw;

    //Sets variables
    double armPos = 0;
    double distToPos = 0;
   //double armPosfreeze;
    //boolean armFree = false;
    double motorSpeed = 0.8;
    //boolean isMoving = false;
    //double armPosfreeze = 0.35;

    private long lastUpdateTime = 0; // Store the last time the variable was updated
    private static final long UPDATE_INTERVAL = 200; // Interval in milliseconds (0.5 seconds)


    @Override
    public void init() {
        //finds components in robot configuration (Motors)
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class,"BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        ArmMotorR = hardwareMap.get(DcMotor.class, "ArmMotorR");
        ArmMotorL = hardwareMap.get(DcMotor.class, "ArmMotorL");
        ArmJoint = hardwareMap.get(DcMotor.class, "ArmJoint");

        //finds components in robot configuration (Servos)
        claw = hardwareMap.get(Servo.class, "claw");

        //sets directions of movement motors
        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        //sets directions of arm motors
        ArmMotorR.setDirection(DcMotorSimple.Direction.FORWARD);
        //ArmMotorL.setDirection(DcMotorSimple.Direction.FORWARD);

        //slows drop of lift kit
        ArmMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //resets encoders in each motor in the lift kit
        ArmMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Sets ArmJoint Encoders
        ArmJoint.setDirection(DcMotor.Direction.FORWARD);
        ArmJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmJoint.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //prints on the drivers hub "Initialization: Initialized"
        telemetry.addData("Initialization:", "Initialized");
        telemetry.update();
    }

    @Override
    //main loop
    public void loop() {

        long currentTime = System.currentTimeMillis();
/*
        if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
            // Update the variable
            armPos = ArmJoint.getCurrentPosition();
            telemetry.addData("Variable updated: ", armPos);

            // Update the last update time
            lastUpdateTime = currentTime;
        }

 */

        //powers drivetrain
        FrontRight.setPower((((gamepad1.right_trigger - gamepad1.left_trigger - gamepad1.left_stick_x)) - (-gamepad1.right_stick_x)) * motorSpeed);
        FrontLeft.setPower((((gamepad1.right_trigger - gamepad1.left_trigger + gamepad1.left_stick_x)) + (gamepad1.right_stick_x)) * motorSpeed);
        BackLeft.setPower((((gamepad1.right_trigger - gamepad1.left_trigger + gamepad1.left_stick_x)) + (-gamepad1.right_stick_x)) * motorSpeed);
        BackRight.setPower((((gamepad1.right_trigger - gamepad1.left_trigger - gamepad1.left_stick_x)) - (gamepad1.right_stick_x)) * motorSpeed);

        //sets speed of motors
        if (gamepad1.triangle) {
            motorSpeed = 0.8;
        } else if (gamepad1.square) {
            motorSpeed = 0.5;
        } else if (gamepad1.cross) {
            motorSpeed = 0.275;
        }
        /*

        }
        //ArmJoint.setPower(armPower * (double) ((gamepad2.dpad_up ? 1.0 : 0.0) - (gamepad2.dpad_down ? 1.0 : 0.0)));


        if (gamepad2.left_stick_y > 0 && armPos < ArmJoint.getCurrentPosition() && ArmJoint.getCurrentPosition() < 88){
            ArmJoint.setPower(0.9);
        } else if (gamepad2.left_stick_y > 0 && ArmJoint.getCurrentPosition() > 130) {
            ArmJoint.setPower(-0.8);
        } else if (gamepad2.left_stick_y < 0 && ArmJoint.getCurrentPosition() > 88 && armPos > ArmJoint.getCurrentPosition()) {
            ArmJoint.setPower(-0.9);
        } else if (gamepad2.left_stick_y < 0 && ArmJoint.getCurrentPosition() < 130){
            ArmJoint.setPower(0.8);
        }else{
            ArmJoint.setPower(0);
        }

        if (gamepad2.left_stick_y != 0) {
            armPos = ArmJoint.getCurrentPosition();

        ArmJoint.setPower(gamepad2.left_stick_y + 0.02 * (armPos - ArmJoint.getCurrentPosition()));
        //ArmJoint.setPower(gamepad2.left_stick_y);

         */
        /*
        //detects button press to set servo mode to free
        if (gamepad2.dpad_left){
            armFree = !armFree;
        }

        //arm free
        if (armFree){
            armPos = -((gamepad2.left_stick_y)/2 - 0.5);
        //arm up position
        }else if (gamepad2.dpad_up){
            armPos = 0.5;
            armPosfreeze = armPos;
        //arm down position
        } else if (gamepad2.dpad_down){
            armPos = 0.146;
            armPosfreeze = armPos;
        //arm start position
        } else if (gamepad2.back){
            armPos = 0.95;
            armPosfreeze = armPos;
        //arm grabbing position
        } else if (gamepad2.left_bumper){
            armPos = 0.05;
        //arm back to last position before bumper
        } else if (!gamepad2.left_bumper){
            armPos = armPosfreeze;
        }

        //moves servo
        ArmJoint.setPosition(armPos);
        */

        //closes or opens claw
        if (gamepad2.right_bumper || 0 < gamepad2.right_trigger) {
            claw.setPosition(0.5);
        }
        else{
            claw.setPosition(0.8);
        }


        distToPos = armPos - ArmJoint.getCurrentPosition();
        armPos = ArmJoint.getCurrentPosition() + distToPos + 1 * gamepad2.left_stick_y;
        if (gamepad2.a){
            distToPos = 0;
        }

        if (armPos > ArmJoint.getCurrentPosition()) {
            ArmJoint.setPower(Math.pow(1.03, 1.5 * (armPos - ArmJoint.getCurrentPosition())) - 1);
        } else {
            ArmJoint.setPower(-Math.pow(1.03, 1.2 * (ArmJoint.getCurrentPosition() - armPos)) + 1);
        }
        //manual control
        if (!ArmMotorR.isBusy()) {
            //Stop both motors when target position is reached
            ArmMotorR.setPower(gamepad2.right_stick_y * 0.8);
            // Set motors back to RUN_USING_ENCODER mode for other operations
            ArmMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


        if (!ArmMotorL.isBusy()) {
            //Stop both motors when target position is reached
            ArmMotorL.setPower(gamepad2.right_stick_y);
            // Set motors back to RUN_USING_ENCODER mode for other operations
            ArmMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        /*
        if(gamepad2.dpad_up){
            armPos = 214;
        }
        if(gamepad2.dpad_left){
            armPos = 209;
        }

         */


        telemetry.addData("armPos:",armPos);
        telemetry.addData("distToPos:",distToPos);
        telemetry.addData("gamepad2.left_stick_y:",gamepad2.left_stick_y);
        telemetry.addData("Arm Power:",0.02 * (armPos - ArmJoint.getCurrentPosition()));

        // Example telemetry for servo position
        //telemetry.addData("Trigger value: ", gamepad2.left_stick_y / 2 + armPos);
        telemetry.update();
    }
}


