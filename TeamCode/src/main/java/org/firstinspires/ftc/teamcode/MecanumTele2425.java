package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class MecanumTele2425 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("LFMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("LBMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("RFMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("RBMotor");

        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        DcMotor leftSlideMotor = hardwareMap.dcMotor.get("leftSlideMotor");
        DcMotor rightSlideMotor = hardwareMap.dcMotor.get("slideMotor");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        double powerMultiplier = 1.0; // Start at full power
        boolean isHalfPower = false; // Track power state

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            if (gamepad1.left_bumper) {
                if (!isHalfPower) { // Only change state if it was previously false
                    powerMultiplier = powerMultiplier == 0.25 ? 0.5 : 0.25; // Toggle between 25% and 50% driving powa!
                    isHalfPower = true;
                }
            } else {
                isHalfPower = false; // Reset when bumper is released
            }

            if (gamepad1.right_bumper) {
                powerMultiplier = 1.0; // Set to full powa!
            }

            // Intake Motor Controls -- Triangle / X
            if(gamepad1.triangle) {
               intakeMotor.setPower(1);
            }
            if(gamepad1.x){
                intakeMotor.setPower(-1);
            }

            // leftSlideMotor Controls
            if(gamepad1.dpad_left){
                leftSlideMotor.setPower(1);
            }
            if(gamepad1.dpad_right){
                leftSlideMotor.setPower(-1);
            }

            // rightSlideMotor Controls
            if(gamepad1.square){
                rightSlideMotor.setPower(1);
            }
            if(gamepad1.circle){
                rightSlideMotor.setPower(-1);
            }

            frontLeftMotor.setPower(frontLeftPower * powerMultiplier);
            backLeftMotor.setPower(backLeftPower * powerMultiplier);
            frontRightMotor.setPower(frontRightPower * powerMultiplier);
            backRightMotor.setPower(backRightPower * powerMultiplier);
        }
    }
}
