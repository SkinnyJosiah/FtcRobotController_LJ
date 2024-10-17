// Hello, how are you today!!
// I'm gonna mess this up BAD
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
        DcMotor leadMotor = hardwareMap.dcMotor.get("leadMotor"); // for Lead Screw
        DcMotor spoolMotor = hardwareMap.dcMotor.get("spoolMotor");

        //Slide motor framework sec01
        DcMotor rSlideMotor = hardwareMap.dcMotor.get("RightSlideMotor");
        DcMotor lSlideMotor = hardwareMap.dcMotor.get("LeftSlideMotor");

        //Slide motor framework sec02
        //Reverse ONE of the slide motors here so they both go up with a positive value as they are mirrored.

        // Servos for Claw / Linear Slide
        // Connected to Expansion Hub

        Servo ServoTilt = hardwareMap.servo.get("ServoTilt");
        Servo ServoLeftClaw = hardwareMap.servo.get("ServoLeftClaw");
        Servo ServoRightClaw = hardwareMap.servo.get("ServoRightClaw");
        // Servo aimServo = hardwareMap.servo.get("aimServo");
        Servo launchServo = hardwareMap.servo.get("launchServo");

        waitForStart();

        if (isStopRequested()) return;

        boolean halfPower = false; // Activation of robot slow movement.

        while (opModeIsActive()) {


            double gamepadX, gamepadY, rx;

            // Gamepad Stick Function f------------------------------------------------------------f

            gamepadX = -gamepad1.left_stick_x;
            gamepadY = gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;

            double gamepadYYleft = gamepad2.left_stick_y;
            double gamepadYYright = gamepad2.right_stick_y;

            // Gamepad Stick Function f------------------------------------------------------------f

            double joystickAngle = Math.toDegrees(Math.atan2(gamepadY, gamepadX)); // Tracks the angle of the Joystick using arc tangent. Not really needed.

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

            double ClawPos = ServoTilt.getPosition();

            telemetry.addData("Claw Tilt Servo Position", ClawPos);

//          Gamepad Misc Buttons b-----------------------------------------------------------------b



//          \\--//
//          //--\\



//          Setting power to Motors m--------------------------------------------------------------m

            double denominator = Math.max(Math.abs(gamepadY) + Math.abs(gamepadX) + Math.abs(turn), 1);

            double powerMultiplier = halfPower ? 0.5 : 1.0; // changes between half power and full power.

            double LFPOWER = (gamepadY + gamepadX + turn) / denominator;
            double LBPOWER = (gamepadY - gamepadX + turn) / denominator;
            double RFPOWER = (gamepadY - gamepadX - turn) / denominator;
            double RBPOWER = (gamepadY + gamepadX - turn) / denominator;

            LFMotor.setPower(LFPOWER * powerMultiplier);
            LBMotor.setPower(-LBPOWER * powerMultiplier);
            RFMotor.setPower(-RFPOWER * powerMultiplier);
            RBMotor.setPower(-RBPOWER * powerMultiplier);

            // Use gamepad bumpers to set power.

            if (leftBumper) {
                // Set to half power
                halfPower = true;
            } else if (rightBumper) {
                // Set to full power
                halfPower = false;
            }

//          Misc Functions Motors m----------------------------------------------------------------m

//          Intake Motor Controls

            // Control the lead screw based on Triangle and Cross

            if (crossD) {
                // Lead Screw retracts.
                leadMotor.setPower(1.0);
            } else if (triangleD) {
                // Lead Screw goes up, extends hook.
                leadMotor.setPower(-1.0);
            } else {
                // Stopped upon nothing pressed to hold position.
                leadMotor.setPower(0.0);
            }

            // Control the linearMotor based on gamepad2's leftstickY, or in other words, gamepadYYleft.

            double linearPOWER = gamepadYYright / denominator;

            linearMotor.setPower(linearPOWER);

            //double minServoPosition = 0.0;  // Set to the minimum allowed position
            //double maxServoPosition = 1.0;  // Set to the maximum allowed position

            //double tiltPOWER = gamepadYYleft / denominator;

            // Set the tiltPOWER to the specified range. 0,1.
            //tiltPOWER = Math.max(minServoPosition, Math.min(maxServoPosition, tiltPOWER));

            // Set the servo position
            //ServoTilt.setPosition(tiltPOWER);

//          Misc Functions Motors m----------------------------------------------------------------m

            // Servos Tilt, LeftClaw, and RightClaw are for the Linear Slide and Claw.

            // Servos Aim and Launch are for the Crossbow Launcher.

//          Servo Controls

            // Servo for the claw's tilt controls, angling the claw.

            /*
            The backboard is told to be at 30 degrees.
             */


            // Temporary servo fix.

            if (dpadLeft) {
                // set servo to its flat position
                ServoTilt.setPosition(0.55); //
            } else if (circle) {
                // set servo to its backboard position
                ServoTilt.setPosition(-1); //
            } else if (rightButton) {
                // intended to set servo back to hiding position
                ServoTilt.setPosition(0);
            }

//          // Claw Controls, Left and Right claw open

            /*
            Left Claw
             */

            if (dpadUp) {
                // move clockwise, closing (Left) claw.
                ServoLeftClaw.setPosition(1);
            } else if (dpadDown){
                // move counter-clockwise, opening (Left) claw.
                ServoLeftClaw.setPosition(0.1);
            } else {
                ServoLeftClaw.setPosition(0.5);
            }

             /*
            Right Claw
             */

            if (triangle) {
                // move clockwise, closing (Right) claw.
                ServoRightClaw.setPosition(0.1);
            } else if(cross) {
                // move counter-clockwise, opening (Right) claw.
                ServoRightClaw.setPosition(1);
            } else {
                ServoRightClaw.setPosition(0.5);
            }

//          Aim Crossbow Servo + Launch Controls

            /*
            if (rightBumperD) {
                 move to 0 degrees.
                aimServo.setPosition(1);
            } else if (leftBumperD) {
                 move to 0 degrees
                aimServo.setPosition(0.5);
            }
            */

             /*
            Drone Launching Servo
             */

            if (squareD) {
                // move to 0 degrees.
                launchServo.setPosition(0);
            } else if (circleD) {
                // move to 0 degrees
                launchServo.setPosition(1.2);
            } else {
                launchServo.setPosition(0.5);
            }
        }
    }
}
