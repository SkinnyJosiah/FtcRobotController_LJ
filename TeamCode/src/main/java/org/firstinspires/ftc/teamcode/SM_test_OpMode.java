package org.firstinspires.ftc.teamcode;
import static android.provider.SyncStateContract.Helpers.update;

import android.widget.Switch;

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
//PLEASEPLEASE
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class SM_test_OpMode extends LinearOpMode {
    private enum IntakeAndOuttake{
        RETRACTING,
        IDLE,
        EXTEND,
        ABTTORET,
        SCORE,
        XFER,
        SLIDE_UP_AND_STAY,
        SLIDE_DOWN_AND_STAY,
        OFF_THE_WALL,
        DROP_TOP, GRAB
    }
    private IntakeAndOuttake IOState = IntakeAndOuttake.IDLE;
    private ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_BILDA = 28;
    static final double LINEAR_SLIDE_GEARING = 	19.20;
    static final double PULLEY_WHEEL_DIAMETER_INCHES = 1.45;
    static final double COUNTS_PER_LS_INCH = (COUNTS_PER_MOTOR_BILDA / LINEAR_SLIDE_GEARING) * (PULLEY_WHEEL_DIAMETER_INCHES * 3.1415 * 2);
    static final double LINEAR_SLIDE_SPEED = 1.0;
    DcMotor rightSlideMotor = null;
    DcMotor leftSlideMotor = null;
    DcMotor frontLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor backRightMotor = null;
    Servo leftIntake = null;
    Servo rightIntake = null;
    Servo horSlideLeft = null;
    Servo horSlideRight = null;
    Servo intakeTilt = null;
    Servo intake = null;
    Servo xfer = null;
    double lasttime = 0;
    boolean xferready = false;

    void update(){
        switch (IOState){
            case IDLE:
                intakeTilt.setPosition(0.75); //upload stuff
                intake.setPosition(0);
                rightIntake.setPosition(0.06);
                leftIntake.setPosition(0.94);
                xfer.setPosition(0.15);
                break;

            case ABTTORET:
                intakeTilt.setPosition(1);
                if (runtime.seconds()-lasttime>0.5) {
                    intake.setPosition(0.75);
                }
                break;

            case XFER:
                xfer.setPosition(0);
                intakeTilt.setPosition(0.34);
                if (runtime.seconds()-lasttime>0.5) {
                    intake.setPosition(0);
                }
                break;

            case RETRACTING:
                rightIntake.setPosition(0.06);
                leftIntake.setPosition(0.94);
                intakeTilt.setPosition(0);

                if(runtime.seconds()-lasttime>0.85){
                    xfer.setPosition(0.30);
                    }

                if (runtime.seconds()-lasttime>0.5) {
                    horSlideLeft.setPosition(0.33);
                    horSlideRight.setPosition(0.73);
                    }
                break;

            case SCORE:
                rightIntake.setPosition(0.70);
                leftIntake.setPosition(0.30);
                if (runtime.seconds()-lasttime>0.6) {
                    xfer.setPosition(0.15);
                }
                break;

            case EXTEND:
                intakeTilt.setPosition(0.5); //upload stuff
                intake.setPosition(0);
                rightIntake.setPosition(0.06);
                leftIntake.setPosition(0.94);
                xfer.setPosition(0.15);
                horSlideLeft.setPosition(0.097);
                horSlideRight.setPosition(0.973);
                break;

            case OFF_THE_WALL:
                xfer.setPosition(1);
                rightIntake.setPosition(0.95); // top intake right
                leftIntake.setPosition(0.05); // top intake left
                break;

            case DROP_TOP:
                xfer.setPosition(0);
                break;

            case GRAB:
                xfer.setPosition(1);
                if (runtime.seconds()-lasttime>0.6) {
                    rightIntake.setPosition(0.70);
                    leftIntake.setPosition(0.30);
                    intake.setPosition(0);
                }
        }
    }//
    @Override
    public void runOpMode() throws InterruptedException {

        // Driving Motors
        frontLeftMotor = hardwareMap.dcMotor.get("LFMotor");
        backLeftMotor = hardwareMap.dcMotor.get("LBMotor");
        frontRightMotor = hardwareMap.dcMotor.get("RFMotor");
        backRightMotor = hardwareMap.dcMotor.get("RBMotor");
        // Misc. Motors
        leftSlideMotor = hardwareMap.dcMotor.get("leftSlideMotor");
        rightSlideMotor = hardwareMap.dcMotor.get("rightSlideMotor");
        //encoders
        // Reverse Motor direction for proper driving
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //encoders
        // Servos
        leftIntake = hardwareMap.servo.get("leftIntake");
        rightIntake = hardwareMap.servo.get("rightIntake");
        horSlideLeft = hardwareMap.servo.get("horSlideLeft");
        horSlideRight = hardwareMap.servo.get("horSlideRight");
        intakeTilt = hardwareMap.servo.get("intakeTilt");
        intake = hardwareMap.servo.get("intake");
        xfer = hardwareMap.servo.get("xfer");

        waitForStart();
        if (isStopRequested()) return;

        double powerMultiplier = 1.0; // Start at full power
        boolean isHalfPower = false; // Track power state
        runtime.reset();

        while (opModeIsActive()) {
            update();
            //machine.update();
            double y = -gamepad2.left_stick_y;
            double x = -gamepad2.right_stick_x;
            double rx = -gamepad2.left_stick_x;
            boolean intakeToggle = false;
            boolean prevTri = false;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (-y + x + rx) / denominator;
            double backLeftPower = (-y - x + rx) / denominator;
            double frontRightPower = (-y - x - rx) / denominator;
            double backRightPower = (-y + x - rx) / denominator;

            if (gamepad2.left_bumper) {
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

            if (gamepad1.dpad_left) {
                runtime.reset();
                IOState = IntakeAndOuttake.DROP_TOP;
            }

            if (gamepad1.square){
                IOState = IntakeAndOuttake.EXTEND;
            }
            if (gamepad1.triangle){
                runtime.reset();
                IOState = IntakeAndOuttake.ABTTORET;
            }
            if (gamepad1.circle){
                runtime.reset();
                IOState = IntakeAndOuttake.RETRACTING;
            }
            if (gamepad1.cross){
                runtime.reset();
                IOState = IntakeAndOuttake.XFER;
            }
            if (gamepad1.right_bumper){
                runtime.reset();
                IOState = IntakeAndOuttake.SCORE;
            }
            if (gamepad1.dpad_down){
                runtime.reset();
                IOState = IntakeAndOuttake.OFF_THE_WALL;
            }
            if (gamepad1.dpad_up){
                runtime.reset();
                IOState = IntakeAndOuttake.GRAB;
            }

            double slidePowerUp = gamepad1.left_trigger;  // Get the right trigger value (0.0 to 1.0)
            double slidePowerDown = gamepad1.right_trigger; // Get the left trigger value (0.0 to 1.0)

            // hopefully up loooool
            // move both slides at the same time to make slide NOT crooked.
            // If the right trigger is pressed, move the slides up proportionall
            if (slidePowerUp > 0) {
                rightSlideMotor.setPower(-slidePowerUp);
                leftSlideMotor.setPower(slidePowerUp);
            }//commit msg

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