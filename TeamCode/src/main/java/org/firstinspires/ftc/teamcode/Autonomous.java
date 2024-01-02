package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends LinearOpMode {

    protected DcMotor LFMotor;
    protected DcMotor LBMotor;
    protected DcMotor RFMotor;
    protected DcMotor RBMotor;

    @Override
    public void runOpMode() throws InterruptedException{
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");

        // LF MOTOR
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // LB MOTOR
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // RF MOTOR
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // RB MOTOR
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // REVERSE RIGHT SIDE MOTORS (very important, otherwise robot is erratic)
        // If robot moves backwards when commanded to go forwards, reverse the left side instead.
        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);



        waitForStart();

        while (opModeIsActive()) {
            // These are the motors' positions.
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
        }
    }
}
