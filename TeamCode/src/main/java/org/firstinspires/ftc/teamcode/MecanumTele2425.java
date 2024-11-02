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
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);// just fyi, default is forward so you really dont need these lines
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servos
        Servo leftIntake = hardwareMap.servo.get("leftIntake");
        Servo rightIntake = hardwareMap.servo.get("rightIntake");
        Servo spoolServo = hardwareMap.servo.get("spoolServo");

        //          Gamepad Misc Buttons b-----------------------------------------------------------------b
        // what are you trying to do this line to line 89? this will assign the values "dpadUp" etc to whatever that button 
        //is only on init, i can see what you are trying to do and make it more concise but if you want to do this I would          recommend doing it with functions
        /*
        boolean dpadUp(){// function is called dpadUp and will return a type of boolean
        return gamepad1.dpad_up;
        }
        */
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
            double y = gamepad1.left_stick_y; // - removed for reason below
            double x = -gamepad1.right_stick_x;
            double rx = -gamepad1.left_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator; // on this line to the next 4 you can remove a - in front of the y as you already negated it above
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            if (gamepad1.left_bumper) {
                /*
                Here, you could also do
                isHalfPower= !isHalfPower // reset it
                if (isHalfPower){powerMultiplier = 0.5;}else{powerMultiplier=1.0;}
                This solution provides for readability of your code for everyone who might be reading it, which is extremely important when you have multiple people working on the same code
                */
                if (!isHalfPower) { // Only change state if it was previously false
                    powerMultiplier = powerMultiplier == 0.5 ? 1.0 : 0.5; // Toggle between 25% and 50% driving powa!
                    // shouldnt this be 0.5 and 1.0 power in the comment above?
                    isHalfPower = true;
                }
            } else {
                isHalfPower = false; // Reset
            }

            if(gamepad1.dpad_left) {
                horizontalSlideMotor.setPower(1);
            }else if(gamepad1.dpad_right) { // this should use an else to prevent clipping
                horizontalSlideMotor.setPower(-1);
            }// should also contain an else setPower(0) to stop

            if (gamepad1.right_bumper) { // can this be implemented into the if statements above? all this does is set the power to 1 and doesnt reference your "isHalfPower" variable
                powerMultiplier = 1.0; // Set to full powa!
            }

            // Intake Motor Controls -- Triangle / X
            if(gamepad1.triangle) {
               intakeMotor.setPower(1);
            }// again here I would use the else if and then it will allow you to reverse it as well
            if(gamepad1.x){
                intakeMotor.setPower(0);
            }
            
            if(gamepad1.dpad_down){
                spoolServo.setPosition(5);
            }
            if(gamepad1.dpad_up) {
                spoolServo.setPosition(5);
            }
            
            double slidePowerUp = gamepad1.right_trigger;  // Get the right trigger value (0.0 to 1.0)
            double slidePowerDown = gamepad1.left_trigger; // Get the left trigger value (0.0 to 1.0)

            // hopefully up
            // move both slides at the same time to make slide NOT crooked.
            // If the right trigger is pressed, move the slides up proportionally
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
            // ideally you should do everything for motor power in the same block, ie. move it up after setting all these powers (just to make it more concise

        }
    }
}