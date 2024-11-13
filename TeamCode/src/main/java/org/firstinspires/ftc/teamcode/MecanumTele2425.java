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
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Servos
        Servo leftIntake = hardwareMap.servo.get("leftIntake");
        Servo rightIntake = hardwareMap.servo.get("rightIntake");
        Servo spoolServo = hardwareMap.servo.get("spoolServo");
        Servo testServo = hardwareMap.servo.get("testServo");

        //          Gamepad Misc Buttons b-----------------------------------------------------------------b

        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;

        boolean triangle = gamepad1.triangle;
        boolean cross = gamepad1.cross;
        boolean circle = gamepad1.circle;
        boolean square = gamepad1.square;

        boolean leftButton = gamepad1.left_stick_button;
        boolean rightButton = gamepad1.right_stick_button;

        boolean leftBumper = gamepad1.left_bumper;
        boolean rightBumper = gamepad1.right_bumper;

        boolean touchpad = gamepad1.touchpad;

//          Gamepad 2 Controls, Shared names have a D after them to represent "double" or "dos" for the 2nd gamepad.

        boolean dpadUpD = gamepad2.dpad_up;
        boolean dpadDownD = gamepad2.dpad_down;
        boolean dpadLeftD = gamepad2.dpad_left;
        boolean dpadRightD = gamepad2.dpad_right;

        boolean triangleD = gamepad2.triangle;
        boolean crossD = gamepad2.cross;
        boolean circleD = gamepad2.circle;
        boolean squareD = gamepad2.square;

        boolean leftButtonD = gamepad2.left_stick_button;
        boolean rightButtonD = gamepad2.right_stick_button;

        boolean leftBumperD = gamepad2.left_bumper;
        boolean rightBumperD = gamepad2.right_bumper;

        waitForStart();

        if (isStopRequested()) return;

        double powerMultiplier = 1.0; // Start at full power
        boolean isHalfPower = false; // Track power state

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (-y + x + rx) / denominator;
            double backLeftPower = (-y - x + rx) / denominator;
            double frontRightPower = (-y - x - rx) / denominator;
            double backRightPower = (-y + x - rx) / denominator;

            if (gamepad1.left_bumper) {
                if (!isHalfPower) { // Only change state if it was previously false
                    powerMultiplier = powerMultiplier == 0.25 ? 0.5 : 0.25; // Toggle between 25% and 50% driving powa!
                    isHalfPower = true;
                }
            } else {
                isHalfPower = false; // Reset
            }

            if (gamepad1.dpad_left) {
                horizontalSlideMotor.setPower(1);
            }

            if (gamepad1.dpad_right) {
                horizontalSlideMotor.setPower(-1);
            }

            if (gamepad1.right_bumper) {
                powerMultiplier = 1.0; // Set to full powa!
            }

            // Intake Motor Controls -- Triangle / X
            if (gamepad1.triangle) {
                intakeMotor.setPower(1);
            }

            if (gamepad1.x) {
                intakeMotor.setPower(-1);
            }

            double slidePowerUp = gamepad1.right_trigger;  // Get the right trigger value (0.0 to 1.0)
            double slidePowerDown = gamepad1.left_trigger; // Get the left trigger value (0.0 to 1.0)

            // hopefully up
            // move both slides at the same time to make slide NOT crooked.
            // If the right trigger is pressed, move the slides up proportionally
            if (slidePowerUp > 0) {
                rightSlideMotor.setPower(slidePowerUp);
                leftSlideMotor.setPower(-slidePowerUp);
            } else if (slidePowerDown > 0) {
                rightSlideMotor.setPower(-slidePowerDown);
                leftSlideMotor.setPower(slidePowerDown);
            } else {
                rightSlideMotor.setPower(0);
                leftSlideMotor.setPower(0);
            }

            // Set power for the motors
            frontLeftMotor.setPower(frontLeftPower * powerMultiplier);
            backLeftMotor.setPower(backLeftPower * powerMultiplier);
            frontRightMotor.setPower(frontRightPower * powerMultiplier);
            backRightMotor.setPower(backRightPower * powerMultiplier);

            // Servo Work

            if (dpadUpD) {
                spoolServo.setDirection(Servo.Direction.FORWARD);
            }
            if (dpadDownD) {
                spoolServo.setDirection(Servo.Direction.REVERSE);
            }

            if (dpadLeftD) {
                leftIntake.setPosition(-1);
            }
            if (dpadRightD) {
                leftIntake.setPosition(1);
            }

            if (squareD) {
                rightIntake.setPosition(-1);
            }
            if (circleD) {
                rightIntake.setPosition(-1);
            }

            if (triangleD) {
                testServo.setPosition(-1);
            }
            if (crossD) {
                testServo.setPosition(1);
            }
        }
    }
}