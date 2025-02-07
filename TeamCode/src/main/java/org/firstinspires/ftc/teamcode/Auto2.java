package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Auto2", group="Robot")
public class Auto2 extends LinearOpMode {


    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;

    double[] rpos = {0, 0}; //robot position (x, y)
    double[] trpos = {0, 0}; //target position (x, y)
    double[] wpos = {0, 0, 0, 0}; //wheel positions (fl, fr, bl, br)
    double[] dwpos = {0, 0, 0, 0}; //change in wheel positions (fl, fr, bl, br)
    boolean running = true;
    String direction = "Forward";

    @Override
    public void runOpMode() {
        FrontLeft  = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft  = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");

        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setTargetPosition(0);
        FrontRight.setTargetPosition(0);
        BackLeft.setTargetPosition(0);
        BackRight.setTargetPosition(0);

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();


        rpos[0] = 0;
        rpos[1] = 0;
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
        telemetry.addData("Start: ", "True");
        telemetry.update();
        while (opModeIsActive()) {

            dwpos[0] = FrontLeft.getCurrentPosition() - wpos[0];
            dwpos[1] = FrontRight.getCurrentPosition() - wpos[1];
            dwpos[2] = BackLeft.getCurrentPosition() - wpos[2];
            dwpos[3] = BackRight.getCurrentPosition() - wpos[3];

            if (direction.equals("Forward")){
                FrontLeft.setTargetPosition((int) trpos[0]);
                FrontRight.setTargetPosition((int) trpos[0]);
                BackLeft.setTargetPosition((int) trpos[0]);
                BackRight.setTargetPosition((int) trpos[0]);

                rpos[0] += (dwpos[0] + dwpos[1] + dwpos[2] + dwpos[3])/4;
            } else if (direction.equals("Strafing")) {
                FrontLeft.setTargetPosition((int) trpos[1]);
                FrontRight.setTargetPosition((int) -trpos[1]);
                BackLeft.setTargetPosition((int) -trpos[1]);
                BackRight.setTargetPosition((int) trpos[1]);

                rpos[1] += (dwpos[0] + dwpos[3] - dwpos[2] - dwpos[1])/8;
            }

            wpos[0] = FrontLeft.getCurrentPosition();
            wpos[1] = FrontLeft.getCurrentPosition();
            wpos[2] = FrontLeft.getCurrentPosition();
            wpos[3] = FrontLeft.getCurrentPosition();

            telemetry.addData("wpos fl: ", wpos[0]);
            telemetry.addData("wpos fr: ", wpos[1]);
            telemetry.addData("wpos bl: ", wpos[2]);
            telemetry.addData("wpos br: ", wpos[3]);
            telemetry.addData("x: ", rpos[0]);
            telemetry.addData("y; ", rpos[1]);
            telemetry.update();
        }
    }
}
