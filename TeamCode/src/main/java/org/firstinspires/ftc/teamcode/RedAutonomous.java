package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
// hoya
@Autonomous
public class RedAutonomous extends LinearOpMode {

    // Define Motors
    protected DcMotor LFMotor;
    protected DcMotor LBMotor;
    protected DcMotor RFMotor;
    protected DcMotor RBMotor;

    protected DcMotor leadMotor;

    protected DcMotor odoPodLeft;

    protected DcMotor odoPodRight;

    protected DcMotor odoPodAux;

    // Unused Voltage Sensor
    protected VoltageSensor VoltageSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        // Redefine and make motors work.
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");

        leadMotor = hardwareMap.get(DcMotor.class, "leadMotor");

        // Shadow Odometry pod encoders
        odoPodLeft = LFMotor;
        odoPodRight = RFMotor;
        odoPodAux = LBMotor;

        // Voltage Sensor and current Volts
        VoltageSensor = hardwareMap.voltageSensor.iterator().next();
        double Volts = VoltageSensor.getVoltage();

        // Servos for claw
        Servo ServoTilt = hardwareMap.servo.get("ServoTilt");
        Servo ServoLeftClaw = hardwareMap.servo.get("ServoLeftClaw");
        Servo ServoRightClaw = hardwareMap.servo.get("ServoRightClaw");

        // Reset all positions back to 0 after every use.
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            // Add motor telemetry data
            double LFPosition = LFMotor.getCurrentPosition();
            double LBPosition = LBMotor.getCurrentPosition();
            double RFPosition = RFMotor.getCurrentPosition();
            double RBPosition = RBMotor.getCurrentPosition();

            telemetry.addData("Left Front Motor Position", LFPosition);
            telemetry.addData("Left Back Motor Position", LBPosition);
            telemetry.addData("Right Front Motor Position", RFPosition);
            telemetry.addData("Right Back Motor Position", RBPosition);
            telemetry.update();

            // Power based off of voltage and scale
            double power = 0.28;

            // Moving stuff!!
            moveForward(power, 1980);
            stopMotors(100);
            moveBackward(power, 500);
            stopMotors(1000);
            moveRight(power, 500);
            stopMotors(100);
            moveBackward(power, 2000);

            moveRight(power, 6000);

            stopMotors(100);

            stopMotors(30000);

            telemetry.addLine("Waiting for start");
            telemetry.update();

            while (opModeIsActive()) {
                telemetry.update();
                sleep(100);
            }

            requestOpModeStop();
        }
    }

    // Functions for moving
    private void moveForward(double power, long motionTime) {
        LFMotor.setPower(-power);
        LBMotor.setPower(power);
        RFMotor.setPower(-power);
        RBMotor.setPower(-power);
        sleep(motionTime);
    }

    private void moveRight(double power, long motionTime) {
        LFMotor.setPower(-power);
        LBMotor.setPower(-power);
        RFMotor.setPower(power);
        RBMotor.setPower(-power);
        sleep(motionTime);
    }

    private void moveLeft(double power, long motionTime) {
        LFMotor.setPower(power);
        LBMotor.setPower(power);
        RFMotor.setPower(-power);
        RBMotor.setPower(power);
        sleep(motionTime);
    }

    private void moveBackward(double power, long motionTime) {
        LFMotor.setPower(power);
        LBMotor.setPower(-power);
        RFMotor.setPower(power);
        RBMotor.setPower(power);
        sleep(motionTime);
    }

    private void stopMotors(long motionTime) {
        LFMotor.setPower(0);
        LBMotor.setPower(0);
        RFMotor.setPower(0);
        RBMotor.setPower(0);
        sleep(motionTime);
    }

    private void RFMove(double power, long motionTime) {
        LFMotor.setPower(0);
        LBMotor.setPower(0);
        RFMotor.setPower(power);
        RBMotor.setPower(0);
        sleep(motionTime);
    }
}
