package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BucketAutonomous extends LinearOpMode {

    Servo intakeTilt = null;
    Servo intake = null;
    Servo xfer = null;
    Servo leftIntake = null;
    Servo rightIntake = null;

    DcMotor rightSlideMotor = null;
    DcMotor leftSlideMotor = null;

    @Override
    public void runOpMode() {
        // Initialize hardware
        intakeTilt = hardwareMap.get(Servo.class, "intakeTilt");
        intake = hardwareMap.get(Servo.class, "intake");
        xfer = hardwareMap.get(Servo.class, "xfer");
        leftIntake = hardwareMap.get(Servo.class, "leftIntake");
        rightIntake = hardwareMap.get(Servo.class, "rightIntake");

        leftSlideMotor = hardwareMap.dcMotor.get("leftSlideMotor");
        rightSlideMotor = hardwareMap.dcMotor.get("rightSlideMotor");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set servo positions before the robot starts moving-- init
        intakeTilt.setPosition(0.3);
        intake.setPosition(0); // intake

        xfer.setPosition(0);
        rightIntake.setPosition(0.25); // top intake right
        leftIntake.setPosition(0.75); // top intake left

        Trajectory backShort = drive.trajectoryBuilder(new Pose2d())
                .back(20)
                .build();

        Trajectory strafeLeftShort = drive.trajectoryBuilder(backShort.end())
                .strafeLeft(5)
                .build();

        // Wait for the start signal
        waitForStart();

        // drive.turn(Math.toRadians(90)) is how you turn, example.

        if (isStopRequested()) return;

        // Strafe left a little bit so we don't get caught on the wall, then go backwards.

        drive.followTrajectory(strafeLeftShort);
        drive.followTrajectory(backShort);
        sleep(1000);

        // Slides go up, after 2 seconds (2 seconds to ensure slide goes up) we set the intake servos and then XFER drops the sample.
        // Set the intake servos back not to get caught on the bucket, then bring the motors down.

        leftSlideMotor.setPower(-0.6);
        rightSlideMotor.setPower(0.6);
        sleep(2000);
        rightIntake.setPosition(0.65); // top intake right
        leftIntake.setPosition(0.35); // top intake left
        sleep(1000);
        xfer.setPosition(0.15);
        sleep(1000);
        rightIntake.setPosition(0.06);
        leftIntake.setPosition(0.94);
        sleep(1000);
        leftSlideMotor.setPower(1);
        rightSlideMotor.setPower(-1); // put it down
        sleep(1500);
        leftSlideMotor.setPower(0);
        rightSlideMotor.setPower(0);


    }
}
