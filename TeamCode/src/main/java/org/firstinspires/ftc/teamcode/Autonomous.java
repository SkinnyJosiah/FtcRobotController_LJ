package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class RedAutonomous extends LinearOpMode {

    protected DcMotor LFMotor;
    protected DcMotor LBMotor;
    protected DcMotor RFMotor;
    protected DcMotor RBMotor;

    @Override
    public void runOpMode() throws InterruptedException{

        /*
        Define motors and servos
         */

        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");

        Servo ServoTilt = hardwareMap.servo.get("ServoTilt");
        Servo ServoLeftClaw = hardwareMap.servo.get("ServoLeftClaw");
        Servo ServoRightClaw = hardwareMap.servo.get("ServoRightClaw");

        // LF MOTOR (reset position upon each run)
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // LB MOTOR(reset position upon each run)
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // RF MOTOR(reset position upon each run)
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // RB MOTOR(reset position upon each run)
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // REVERSE RIGHT SIDE MOTORS (very important, otherwise robot is erratic)
        // If robot moves backwards when commanded to go forwards, reverse the left side instead.

        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);



        waitForStart();

        while (opModeIsActive()) {

            // These are the motors' positions, shown on the Driver Hub.
            double LFPosition = LFMotor.getCurrentPosition();
            double LBPosition = LBMotor.getCurrentPosition();
            double RFPosition = RFMotor.getCurrentPosition();
            double RBPosition = RBMotor.getCurrentPosition();

            // Show the position of the motors.
            telemetry.addData("Left Front Motor Position", LFPosition);
            telemetry.addData("Left Back Motor Position", LBPosition);
            telemetry.addData("Right Front Motor Position", RFPosition);
            telemetry.addData("Right Back Motor Position", RBPosition);
            telemetry.update();

            // Move forward. Push purple pixel onto the red tape.
            LFMotor.setPower(1);
            LBMotor.setPower(1);
            RFMotor.setPower(1);
            RBMotor.setPower(1);

            sleep (500);

            // Robot moves backward, to wall.
            LFMotor.setPower(-1);
            LBMotor.setPower(-1);
            RFMotor.setPower(-1);
            RBMotor.setPower(-1);

            sleep(500);

            // Robot goes to the right to park.
            LFMotor.setPower(1);
            LBMotor.setPower(-1);
            RFMotor.setPower(-1);
            RBMotor.setPower(1);

            sleep (1000);

            // Stop when parked.
            LFMotor.setPower(0);
            LBMotor.setPower(0);
            RFMotor.setPower(0);
            RBMotor.setPower(0);

        }
    }
}
