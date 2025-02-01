package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class ParkAutonomous extends LinearOpMode {

    Servo intakeTilt = null;
    Servo intake = null;
    Servo xfer = null;
    Servo leftIntake = null;
    Servo rightIntake = null;
//
    @Override
    public void runOpMode() {
        // Initialize hardware
        intakeTilt = hardwareMap.get(Servo.class, "intakeTilt");
        intake = hardwareMap.get(Servo.class, "intake");
        xfer = hardwareMap.get(Servo.class, "xfer");
        leftIntake = hardwareMap.get(Servo.class, "leftIntake");
        rightIntake = hardwareMap.get(Servo.class, "rightIntake");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set servo positions before the robot starts moving
        rightIntake.setPosition(0.50);
        leftIntake.setPosition(0.50);
        xfer.setPosition(1);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .back(24)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeRight(15)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .forward(20)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj2.end())
                .strafeLeft(20)
                .build();


        // Wait for the start signal
        waitForStart();

        if (isStopRequested()) return;
        drive.followTrajectory(traj2);
        sleep(100);
        drive.followTrajectory(traj1);
        sleep(1000);
        rightIntake.setPosition(0.95); // top intake right
        leftIntake.setPosition(0.05); // top intake left
        sleep(1500);
        xfer.setPosition(0);
        drive.followTrajectory(traj3);
        sleep(1000);
        drive.followTrajectory(traj4);

    }
}
