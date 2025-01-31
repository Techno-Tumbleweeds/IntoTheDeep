package org.firstinspires.ftc.teamcode;

//imports packages
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    Servo clawmove;

    //Sets variables
    double armPos = 0;
    double liftPos = 0;
    double distToPosLift = 0;
    double distToPos = 0;
   //double armPosfreeze;
    //boolean armFree = false;
    double motorSpeed = 0.8;
    //boolean isMoving = false;
    //double armPosfreeze = 0.35;

    double RotatePos = 0.15;

    private long lastUpdateTime = 0; // Store the last time the variable was updated
    private static final long UPDATE_INTERVAL = 200; // Interval in milliseconds (0.5 seconds)

    ElapsedTime buttonTimer = new ElapsedTime();


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
        clawmove = hardwareMap.get(Servo.class, "clawmove");

        //sets directions of movement motors
        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        //sets directions of arm motors
        ArmMotorL.setDirection(DcMotorSimple.Direction.FORWARD);

        //slows drop of lift kit
        ArmMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //resets encoders in each motor in the lift kit
        ArmMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        FrontRight.setPower((((gamepad1.right_trigger - gamepad1.left_trigger - gamepad1.left_stick_x)) - (gamepad1.right_stick_x)) * motorSpeed);
        BackRight.setPower((((gamepad1.right_trigger - gamepad1.left_trigger - gamepad1.left_stick_x)) + (gamepad1.right_stick_x)) * motorSpeed);
        BackLeft.setPower((((gamepad1.right_trigger - gamepad1.left_trigger + gamepad1.left_stick_x)) - (gamepad1.right_stick_x)) * motorSpeed);
        FrontLeft.setPower((((gamepad1.right_trigger - gamepad1.left_trigger + gamepad1.left_stick_x)) + (gamepad1.right_stick_x)) * motorSpeed);
/*
        FrontRight.setPower((((gamepad1.right_trigger - gamepad1.left_trigger - gamepad1.left_stick_x)) + (gamepad1.right_stick_x)) * motorSpeed);
        FrontLeft.setPower((((gamepad1.right_trigger - gamepad1.left_trigger + gamepad1.left_stick_x)) + (gamepad1.right_stick_x)) * motorSpeed);
        BackLeft.setPower((((gamepad1.right_trigger - gamepad1.left_trigger + gamepad1.left_stick_x)) - (gamepad1.right_stick_x)) * motorSpeed);
        BackRight.setPower((((gamepad1.right_trigger - gamepad1.left_trigger - gamepad1.left_stick_x)) - (gamepad1.right_stick_x)) * motorSpeed);

 */
        //sets speed of motors
        if (gamepad1.triangle) {
            motorSpeed = 0.7;
        } else if (gamepad1.square) {
            motorSpeed = 0.55;
        } else if (gamepad1.cross) {
            motorSpeed = 0.27;
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


        //closes or opens claw
        if (gamepad2.right_bumper || 0 < gamepad2.right_trigger) {
            claw.setPosition(-0.3);
            //0.45
        }
        else{
            claw.setPosition(0.25);
        }
        if (gamepad2.left_bumper) {
            claw.setPosition(1);
        }


/*
        distToPos = armPos - ArmJoint.getCurrentPosition();
        armPos = ArmJoint.getCurrentPosition() + distToPos + 3 * gamepad2.left_stick_y;
        if (gamepad2.a){
            distToPos = 0;
        }

        if (armPos > ArmJoint.getCurrentPosition()) {
            ArmJoint.setPower(Math.pow(1.02, 1.5 * (armPos - ArmJoint.getCurrentPosition())) - 1);
        } else {
            ArmJoint.setPower(-Math.pow(1.02, 1.5 * (ArmJoint.getCurrentPosition() - armPos)) + 1);
        }

 */


        /*
        double kP = 0.01; // Tune this value
        double error = armPos - ArmJoint.getCurrentPosition();


// Adjust armPos only when stick is moved significantly
        if (Math.abs(gamepad2.left_stick_y) > 0.05) {
            armPos += 3 * gamepad2.left_stick_y;
        }

// Reset position when button is pressed
        if (gamepad2.a) {
            distToPos = 0;
        }

// Apply smooth power control
        double power = kP * error;
        power = Range.clip(power, -1, 1); // Keep power in range

// Stop small jittering movements
        if (Math.abs(error) < 5) {
            power = 0;
        }*/


/*
        if (gamepad2.x && buttonTimer.milliseconds() > 300) {
            ArmJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmJoint.setTargetPosition(100); // Set target position for X button
            ArmJoint.setPower(0.5);
            buttonTimer.reset();  // Reset the timer after button press
        } else if (gamepad2.y && buttonTimer.milliseconds() > 300) {
            ArmJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmJoint.setTargetPosition(200); // Set target position for Y button
            ArmJoint.setPower(0.5);
            buttonTimer.reset();  // Reset the timer after button press
        } else if (gamepad2.a && buttonTimer.milliseconds() > 300) {
            ArmJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmJoint.setTargetPosition(300); // Set target position for A button
            ArmJoint.setPower(0.5);
            buttonTimer.reset();  // Reset the timer after button press
        } else if (gamepad2.b && buttonTimer.milliseconds() > 300) {
            ArmJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmJoint.setTargetPosition(400); // Set target position for B button
            ArmJoint.setPower(0.5);
            buttonTimer.reset();  // Reset the timer after button press
        } else if (!ArmJoint.isBusy()) {
            ArmJoint.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Return to normal mode
            ArmJoint.setPower(gamepad2.left_stick_y);  // Stop motor when target position is reached
        }else {
            ArmJoint.setPower(gamepad2.left_stick_y);  // Stop motor when target position is reached
        }

 */


        distToPosLift = liftPos - ArmMotorL.getCurrentPosition();
        liftPos = ArmMotorL.getCurrentPosition() + distToPosLift - 40 * gamepad2.right_stick_y;
        if (gamepad2.b){
            distToPosLift = 0;
            telemetry.addData("RESET: ", "movement reset");
        }

        ArmMotorL.setPower(-(liftPos - ArmMotorL.getCurrentPosition())/200);
        telemetry.addData("Arm Power:",(liftPos - ArmMotorL.getCurrentPosition())/200);

        if (gamepad2.y) {
            ArmJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmJoint.setTargetPosition(-100);
            ArmJoint.setPower(0.35);
        } else if (!gamepad2.y) {
            ArmJoint.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ArmJoint.setPower(gamepad2.left_stick_y/2);
        }




/*
        if (!ArmMotorL.isBusy()) {
            //Stop both motors when target position is reached
            ArmMotorL.setPower(gamepad2.right_stick_y);
            // Set motors back to RUN_USING_ENCODER mode for other operations
            ArmMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

*/
        ElapsedTime buttonTimer = new ElapsedTime(); // Timer for button control


        if (gamepad2.dpad_left||gamepad1.dpad_left ) { // 150ms delay
            RotatePos += 0.01; // Increment value
            //RotatePos = Range.clip(RotatePos, 0.5, 1.0); // Clamp value
            buttonTimer.reset(); // Reset timer
        } else if (gamepad2.dpad_right||gamepad1.dpad_right) {
            RotatePos -= 0.01; // Decrement value
            //RotatePos = Range.clip(RotatePos, 0.05, 1.0); // Clamp value
            buttonTimer.reset(); // Reset timer
        }else if(gamepad2.dpad_up||gamepad1.dpad_up){
            RotatePos = 0.48;
        }else if (gamepad2.dpad_down||gamepad1.dpad_down){
            RotatePos = 0.15;
        }
        clawmove.setPosition(RotatePos);



        telemetry.addData("liftPos: ",liftPos);
        telemetry.addData("ActualPos: ", ArmMotorL.getCurrentPosition());
        telemetry.addData("distToPos: ",distToPosLift);
        telemetry.addData("gamepad2.right_stick_y: ",gamepad2.right_stick_y);
        telemetry.addData("Servo Open Pos", claw.getPosition());
        telemetry.addData("Rotate Pos", RotatePos);
        telemetry.addData("ArmJoint", ArmJoint.getCurrentPosition());


        // Example telemetry for servo position
        //telemetry.addData("Trigger value: ", gamepad2.left_stick_y / 2 + armPos);
        telemetry.update();
    }
}


