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
public class MecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Our Motors for Driving
        // Connected to Control Hub
        DcMotor LFMotor = hardwareMap.dcMotor.get("LFMotor");
        DcMotor LBMotor = hardwareMap.dcMotor.get("LBMotor");
        DcMotor RFMotor = hardwareMap.dcMotor.get("RFMotor");
        DcMotor RBMotor = hardwareMap.dcMotor.get("RBMotor");

        // REVERSE RIGHT SIDE MOTORS (very important, otherwise robot is erratic)
        // If robot moves backwards when commanded to go forwards, reverse the left side instead.
        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Motor for Intake & Linear Slide
        // Connected to Expansion Hub
        DcMotor linearMotor = hardwareMap.dcMotor.get("linearMotor"); // For Linear Slide
        DcMotor slideMotor = hardwareMap.dcMotor.get("slideMotor"); // for Lead Screw

        // Servos for Claw / Linear Slide
        // Connected to Expansion Hub
        Servo ServoTilt = hardwareMap.servo.get("ServoTilt");
        Servo ServoLeftClaw = hardwareMap.servo.get("ServoLeftClaw");
        Servo ServoRightClaw = hardwareMap.servo.get("ServoRightClaw");
        Servo aimServo = hardwareMap.servo.get("aimServo");
        Servo launchServo = hardwareMap.servo.get("launchServo");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double gamepadX, gamepadY, rx;

            // Gamepad Stick Function f------------------------------------------------------------f

            gamepadX = -gamepad1.left_stick_x;
            gamepadY = gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;

            double gamepadYYleft = gamepad2.left_stick_y;
            double gamepadYYright = gamepad2.right_stick_y;

            // Gamepad Stick Function f------------------------------------------------------------f

            double joystickAngle = Math.toDegrees(Math.atan2(gamepadY, gamepadX)); // Tracks the angle of the Joystick using arc tangent.

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

//          Gamepad Misc Buttons b-----------------------------------------------------------------b

//          \\--//
//          //--\\

//          Setting power to Motors m--------------------------------------------------------------m

            double denominator = Math.max(Math.abs(gamepadY) + Math.abs(gamepadX) + Math.abs(turn), 1);

            double LFPOWER = (gamepadY + gamepadX + turn) / denominator;
            double LBPOWER = (gamepadY - gamepadX + turn) / denominator;
            double RFPOWER = (gamepadY - gamepadX - turn) / denominator;
            double RBPOWER = (gamepadY + gamepadX - turn) / denominator;

            LFMotor.setPower(LFPOWER);
            LBMotor.setPower(LBPOWER);
            RFMotor.setPower(RFPOWER);
            RBMotor.setPower(-RBPOWER);

//          Misc Functions Motors m----------------------------------------------------------------m

//          Intake Motor Controls

            // Control the slideMotor based on Triangle and Cross

            if (crossD) {
                // Turn left
                slideMotor.setPower(1.0); // Adjust the power value as needed
            } else if (triangleD) {
                // Turn right
                slideMotor.setPower(-1.0); // Adjust the power value as needed
            } else {
                // Stop the slideMotor when neither dpad_up nor dpad_down is pressed
                slideMotor.setPower(0.0);
            }

            // Control the linearMotor based on gamepad2's leftstickY, or in other words, gamepadYYleft.

            double linearPOWER = gamepadYYright / denominator;

            linearMotor.setPower(linearPOWER);

            double minServoPosition = 0.0;  // Set to the minimum allowed position
            double maxServoPosition = 1.0;  // Set to the maximum allowed position

            //double tiltPOWER = gamepadYYleft / denominator;

            // Set the tiltPOWER to the specified range. 0,1.
            //tiltPOWER = Math.max(minServoPosition, Math.min(maxServoPosition, tiltPOWER));

            // Set the servo position
            //ServoTilt.setPosition(tiltPOWER);

//          Misc Functions Motors m----------------------------------------------------------------m

            // Servos Tilt, LeftClaw, and RightClaw are for the Linear Slide and Claw.

            // Servos Aim and Launch are for the Crossbow Launcher.

//          Servo Controls

            // Servo for the claw's tilt controls, angling the claw when dpadLeftD is pushed.

            if (dpadRightD) {
                // set servo to it's flat position
                ServoTilt.setPosition(0.25);
            } else if (dpadLeftD) {
                // set servo to it's backboard position
                ServoTilt.setPosition(-0.1);
            }

//          // Claw Controls, Left and Right claw open

            if (dpadUp) {
                // move clockwise, closing (Left) claw.
                ServoLeftClaw.setPosition(1);
            } else if (dpadDown){
                // move c-c, opening (Left) claw.
                ServoLeftClaw.setPosition(0.1);
            } else {
                ServoLeftClaw.setPosition(0.5);
            }

            if (triangle) {
                // move clockwise, closing (Right) claw.
                ServoRightClaw.setPosition(0.1);
            } else if(cross) {
                // move c-c, opening (Right) claw.
                ServoRightClaw.setPosition(1);
            } else {
                ServoRightClaw.setPosition(0.5);
            }

//          Aim Crossbow Servo + Launch Controls

            if (dpadUpD) {
                // move to 0 degrees.
                aimServo.setPosition(1);
            } else if (dpadDownD) {
                // move to 0 degrees
                aimServo.setPosition(0.5);
            }

            if (squareD) {
                // move to 0 degrees.
                launchServo.setPosition(0);
            } else if (circleD) {
                // move to 0 degrees
                launchServo.setPosition(-1);
            }
        }
    }
}





// Unused Code

//          Setting power to Motors m--------------------------------------------------------------m

// Denominator is the largest motor power (absolute value) or 1
// This ensures all the powers maintain the same ratio,
// but only if at least one is out of the range [-1, 1]

// Unused Motor Power Code

//          double frontRightPower = (yControl * Math.abs(yControl) - xControl * Math.abs(xControl) + turn);
//          double backRightPower = (yControl * Math.abs(yControl) + xControl * Math.abs(xControl) + turn);
//          double frontLeftPower = (yControl * Math.abs(yControl) + xControl * Math.abs(xControl) - turn);
//          double backLeftPower = (yControl * Math.abs(yControl) - xControl * Math.abs(xControl) - turn);

//          LFMotor.setPower(frontLeftPower);
//          LBMotor.setPower(backLeftPower);
//          RFMotor.setPower(frontRightPower);
//          RBMotor.setPower(backRightPower);

//          Setting power to Motors m--------------------------------------------------------------m
