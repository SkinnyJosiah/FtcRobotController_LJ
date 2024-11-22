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
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class SM_test_OpMode extends LinearOpMode {
    enum IntakeAndOuttake{
        IDLING,
        EXTENDING,
        DEPOSITING,
        RETRACTING
    }
    private ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_BILDA = 28;
    static final double LINEAR_SLIDE_GEARING = 	19.20;
    static final double PULLEY_WHEEL_DIAMETER_INCHES = 1.45;
    static final double COUNTS_PER_LS_INCH = (COUNTS_PER_MOTOR_BILDA / LINEAR_SLIDE_GEARING) * (PULLEY_WHEEL_DIAMETER_INCHES * 3.1415 * 2);
    static final double LINEAR_SLIDE_SPEED = 1.0;
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
        //encoders
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
        //encoders
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Servos
        Servo leftIntake = hardwareMap.servo.get("leftIntake");
        Servo rightIntake = hardwareMap.servo.get("rightIntake");
        Servo spoolServo = hardwareMap.servo.get("spoolServo");
        Servo meshNet = hardwareMap.servo.get("meshNet");
        Servo horSlideLeft = hardwareMap.servo.get("horSlideLeft");
        Servo horSlideRight = hardwareMap.servo.get("horSlideRight");

        //position stuff
        /*StateMachine machine = new StateMachineBuilder()
                .state(IntakeAndOuttake.IDLING)
                .onEnter( () -> {
                    horSlideLeft.setPosition(0.027);
                    horSlideRight.setPosition(0.973);
                })
                .transition( () -> gamepad1.circle )
                .onExit( () -> {
                    horSlideLeft.setPosition(0.28);
                    horSlideRight.setPosition(0.72);
                })
                /*.state(IntakeAndOuttake.EXTENDING)
                .onEnter( () -> System.out.println( "Entering the second state" ) )
                .transition( () -> gamepad1.b) // if check2 is false transition
                .state(IntakeAndOuttake.DEPOSITING)
                .onEnter( () -> System.out.println( "In the third state " ) )
                .build();*/

        waitForStart();
        //machine.start();
        /*public void LinearSlideDrive(double slideSpeed, double slideRotationInches, double slideTmeoutS){
            int rightSlideTarget;
            int leftSlideTarget;
            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                rightSlideTarget = rightSlideMotor.getCurrentPosition() + (int)(slideRotationInches * COUNTS_PER_LS_INCH);
                leftSlideTarget = leftSlideMotor.getCurrentPosition() + (int)(slideRotationInches * COUNTS_PER_LS_INCH);

                rightSlideMotor.setTargetPosition(rightSlideTarget);
                leftSlideMotor.setTargetPosition(leftSlideTarget);
                // Turn On RUN_TO_POSITION
                rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                rightSlideMotor.setPower(Math.abs(slideSpeed));
                leftSlideMotor.setPower(Math.abs(slideSpeed));

                while (opModeIsActive() && (runtime.seconds() < slideTmeoutS) && (leftSlideMotor.isBusy() && (rightSlideMotor.isBusy()))) {
                    telemetry.update();
                }
                // Stop all motion;
                rightSlideMotor.setPower(0);
                leftSlideMotor.setPower(0);

                // Turn off RUN_TO_POSITION
                rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                sleep(250);   // optional pause after each move.
            }
        }*/
        if (isStopRequested()) return;

        double powerMultiplier = 1.0; // Start at full power
        boolean isHalfPower = false; // Track power state

        while (opModeIsActive()) {
            //machine.update();
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

            if (gamepad1.circle){
                horSlideLeft.setPosition(0.28);
                horSlideRight.setPosition(0.72);
            }
            else if (gamepad1.cross) {
                horSlideLeft.setPosition(0.027);
                horSlideRight.setPosition(0.973);
            }
            double slidePowerUp = gamepad2.right_trigger;  // Get the right trigger value (0.0 to 1.0)
            double slidePowerDown = gamepad2.left_trigger; // Get the left trigger value (0.0 to 1.0)

            // hopefully up loooool
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