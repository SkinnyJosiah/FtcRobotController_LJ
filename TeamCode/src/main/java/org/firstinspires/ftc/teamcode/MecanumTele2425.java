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

        // Driving Motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("LFMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("LBMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("RFMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("RBMotor");

        // Misc. Motors
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        DcMotor horizontalSlideMotor = hardwareMap.dcMotor.get("horizontalSlideMotor");
        DcMotor leftSlideMotor = hardwareMap.dcMotor.get("leftSlideMotor");
        DcMotor rightSlideMotor = hardwareMap.dcMotor.get("rightSlideMotor");

        // Reverse Motor direction for proper driving
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Servos
        Servo leftIntake = hardwareMap.servo.get("leftIntake");
        Servo rightIntake = hardwareMap.servo.get("rightIntake");
        Servo spoolServo = hardwareMap.servo.get("spoolServo");
        Servo meshNet = hardwareMap.servo.get("meshNet");

        waitForStart();

        if (isStopRequested()) return;

        double powerMultiplier = 1.0; // Start at full power
        boolean isHalfPower = false; // Track power state

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.right_stick_x;
            double rx = -gamepad1.left_stick_x;
            boolean intakeToggle = false;
            boolean prevX = false;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (-y + x + rx) / denominator;
            double backLeftPower = (-y - x + rx) / denominator;
            double frontRightPower = (-y - x - rx) / denominator;
            double backRightPower = (-y + x - rx) / denominator;

            if (gamepad1.left_bumper) {
                if (!isHalfPower) { // Only change state if it was previously false
                    powerMultiplier = powerMultiplier == 0.5 ? 1.0 : 0.5; // Toggle between 25% and 50% driving powa!
                    isHalfPower = true;
                }
            } else {
                isHalfPower = false; // Reset
            }

           // if(gamepad2.dpad_left) {
                //horizontalSlideMotor.setPower(-0.3);
            //}

            //if(gamepad2.dpad_right) {
                //horizontalSlideMotor.setPower(0.3);
            //}
            if (gamepad2.right_bumper) {
                powerMultiplier = 1.0; // Set to full powa!
            }
            if (gamepad2.x && !prevX) { // Detects a rising edge on the X button
                intakeToggle = !intakeToggle; // Toggle the state
            }
            prevX = gamepad2.x; // Update the previous state
            intakeMotor.setPower(intakeToggle ? 1.0 : 0.0);

            if(gamepad2.dpad_down){
                spoolServo.setPosition(0.02);
                meshNet.setPosition(0);
            }
                else if (gamepad2.dpad_up) {
                spoolServo.setPosition(0.358);
                meshNet.setPosition(0.5);
                }
            if(gamepad2.a){
                rightIntake.setPosition(0.3);
                leftIntake.setPosition(0.7);
            }
                else if(gamepad2.b){
                    rightIntake.setPosition(1);
                    leftIntake.setPosition(0);
                }
            double slidePowerUp = gamepad2.right_trigger;  // Get the right trigger value (0.0 to 1.0)
            double slidePowerDown = gamepad2.left_trigger; // Get the left trigger value (0.0 to 1.0)

            // hopefully up
            // move both slides at the same time to make slide NOT crooked.
            // If the right trigger is pressed, move the slides up proportionall
            if (slidePowerUp > 0) {
                rightSlideMotor.setPower(-slidePowerUp);
                leftSlideMotor.setPower(slidePowerUp);
            }

            else if (slidePowerDown > 0) {
                rightSlideMotor.setPower(slidePowerDown);
                leftSlideMotor.setPower(-slidePowerDown);
            }

            else {
                rightSlideMotor.setPower(0);
                leftSlideMotor.setPower(0);
            }

            // Set power for the motors
            frontLeftMotor.setPower(frontLeftPower * powerMultiplier);
            backLeftMotor.setPower(backLeftPower * powerMultiplier);
            frontRightMotor.setPower(frontRightPower * powerMultiplier);
            backRightMotor.setPower(backRightPower * powerMultiplier);

        }
    }
}