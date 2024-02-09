package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import javax.lang.model.element.VariableElement;

@Autonomous
public class RedNoParkAutonomous extends LinearOpMode {

    // Define Motors
    protected DcMotor LFMotor;
    protected DcMotor LBMotor;
    protected DcMotor RFMotor;
    protected DcMotor RBMotor;

    protected DcMotor linearMotor;

    private double prevLeftEncoderPos = 0;
    private double prevRightEncoderPos = 0;
    private double prevCenterEncoderPos = 0;

    private double xPosition = 0.0;
    private double yPosition = 0.0;
    private double heading = 0.0;

    // Unused Voltage Sensor
    protected VoltageSensor VoltageSensor;

    //Camera
    OpenCvWebcam webcam;

    @Override
    public void runOpMode() throws InterruptedException {

        // Redefine and make motors work.

        /*
        linearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearMotor.setTargetPosition(1);
         */

        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");

        linearMotor = hardwareMap.get(DcMotor.class, "linearMotor");

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

        // Initialize camera and set up OpenCV pipeline
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new SamplePipeline());
        webcam.setMillisecondsPermissionTimeout(5000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();

        while (opModeIsActive()) {


            telemetry.update();

            // Calculate the scaling factor based off of what previously worked. Unused.
            //double scalingFactor = 0.275 / 12.28;

            // Power based off of voltage and scale
            // This is almost always changing. Refer to https://www.desmos.com/calculator/gyl9bsxfqc

            double power = 0.265;

            linearMotor.setPower(-1);

            sleep(400);

            linearMotor.setPower(0);

            // Moving stuff!!
            moveForward(power, 1980);
            stopMotors(100);
            moveBackward(power, 500);
            stopMotors(1000);
            moveRight(power, 500);
            stopMotors(100);
            moveBackward(power, 2000);
            stopMotors(30000);

            telemetry.addLine("Waiting for start");
            telemetry.update();

            while (opModeIsActive()) {
                telemetry.addData("Frame Count", webcam.getFrameCount());
                telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
                telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
                telemetry.update();

                if (gamepad1.left_stick_button) {
                    webcam.stopStreaming();
                    webcam.closeCameraDevice();
                }

                sleep(100);
            }

            requestOpModeStop();
        }
    }

    // Functions for moving. Self explanatory, just call upon the function and give the first parameter power. motionTime will be removed soon once we have odometry pods and better encoder usage.

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

    // A LOT of camera work.

    class SamplePipeline extends OpenCvPipeline {
        boolean viewportPaused;
        ColorDetection colorDetection;

        public SamplePipeline() {
            colorDetection = new ColorDetection("red");
        }

        @Override
        public Mat processFrame(Mat input) {
            // Process the frame using color detection and saturation check
            Mat processedFrame = colorDetection.processFrame(input);

            // Draw rectangles on the original input frame
            drawRedRectangles(input);

            return processedFrame;
        }

        private void drawRedRectangles(Mat input) {
            // Center Rectangle
            Imgproc.rectangle(
                    input,
                    new Point(input.cols() / 4, input.rows() / 8),
                    new Point(input.cols() * (2f / 4f), input.rows() * (2f / 3f)),
                    new Scalar(128, 0, 0), 2); // Draw the rectangle outline in red

            // Right Rectangle
            Imgproc.rectangle(
                    input,
                    new Point(input.cols() * (3f / 4f), input.rows() / 8),
                    new Point(input.cols() - 1, input.rows() * (3f / 4f)),
                    new Scalar(128, 0, 0), 2); // Draw the rectangle outline in red
        }


        @Override
        public void onViewportTapped() {
            viewportPaused = !viewportPaused;

            if (viewportPaused) {
                webcam.pauseViewport();
            } else {
                webcam.resumeViewport();
            }
        }
    }

        public static class ColorDetection extends OpenCvPipeline {
            Mat hsv = new Mat();
            Mat mask = new Mat();

            String color;

            public ColorDetection(String color) {
                this.color = color;
            }

            @Override
            public Mat processFrame(Mat input) {
                // Process the frame using color detection
                processColorDetection(input);

                // Draw rectangles on the original input frame
                drawRedRectangles(input);

                return input;
            }

            private void processColorDetection(Mat input) {
                Scalar scalarLow, scalarHigh;

                if (color.equals("blue")) {
                    // Blue color range
                    scalarLow = new Scalar(90, 100, 100);
                    scalarHigh = new Scalar(140, 255, 255);
                } else if (color.equals("red")) {
                    // Red color range
                    scalarLow = new Scalar(0, 100, 100);
                    scalarHigh = new Scalar(10, 255, 255);
                } else {
                    // Default color range
                    scalarLow = new Scalar(100, 0, 0);
                    scalarHigh = new Scalar(0, 0, 100);
                }

                // Convert the input frame to the HSV color space
                Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

                // Threshold the HSV image to get only the specified color range
                Core.inRange(hsv, scalarLow, scalarHigh, mask);

                // Check and change color based on the threshold
                checkAndChangeColor(input, mask, 35, 35);
            }

            private void drawRedRectangles(Mat input) {
                // Draw rectangles on the original input frame
                Imgproc.rectangle(
                        input,
                        new Point(input.cols() / 4, input.rows() / 8),
                        new Point(input.cols() * (2f / 4f), input.rows() * (2f / 3f)),
                        new Scalar(128, 0, 0), 2); // Draw the rectangle outline in red

                Imgproc.rectangle(
                        input,
                        new Point(input.cols() * (3f / 4f), input.rows() / 8),
                        new Point(input.cols() - 1, input.rows() * (3f / 4f)),
                        new Scalar(128, 0, 0), 2); // Draw the rectangle outline in red
            }

            private void checkAndChangeColor(Mat input, Mat mask, int redThresholdCenter, int redThresholdRight) {
                Mat temp = new Mat();
                Imgproc.cvtColor(input, temp, Imgproc.COLOR_RGB2BGR); // Convert to BGR color space

                // Counters for red and total pixels in each rectangle
                int redPixelCountCenter = 0;
                int totalPixelCountCenter = 0;

                int redPixelCountRight = 0;
                int totalPixelCountRight = 0;

                for (int i = 0; i < mask.rows(); i++) {
                    for (int j = 0; j < mask.cols(); j++) {
                        double[] pixel = mask.get(i, j);
                        if (pixel[0] > 0) {
                            double redValue = temp.get(i, j)[2]; // BGR: 0 - Blue, 1 - Green, 2 - Red

                            // Check if the pixel is within the center rectangle
                            if (j >= input.cols() / 4 && j <= input.cols() * (2f / 4f) &&
                                    i >= input.rows() / 8 && i <= input.rows() * (2f / 3f)) {
                                totalPixelCountCenter++;

                                if (redValue > 0) {
                                    redPixelCountCenter++;
                                }
                            }

                            // Check if the pixel is within the right rectangle
                            if (j >= input.cols() * (3f / 4f) && j <= input.cols() - 1 &&
                                    i >= input.rows() / 8 && i <= input.rows() * (3f / 4f)) {
                                totalPixelCountRight++;

                                if (redValue > 0) {
                                    redPixelCountRight++;
                                }
                            }
                        }
                    }
                }

                // Calculate the percentage of red pixels in each rectangle
                double percentageRedCenter = (double) redPixelCountCenter / totalPixelCountCenter * 100;
                double percentageRedRight = (double) redPixelCountRight / totalPixelCountRight * 100;

                // Turn the rectangle green only if it has 35% or more red pixels
                if (percentageRedCenter >= redThresholdCenter) {
                    Imgproc.rectangle(
                            input,
                            new Point(input.cols() / 4, input.rows() / 8),
                            new Point(input.cols() * (2f / 4f), input.rows() * (2f / 3f)),
                            new Scalar(0, 255, 0), -1); // Turn the center rectangle green
                }

                if (percentageRedRight >= redThresholdRight) {
                    Imgproc.rectangle(
                            input,
                            new Point(input.cols() * (3f / 4f), input.rows() / 8),
                            new Point(input.cols() - 1, input.rows() * (3f / 4f)),
                            new Scalar(0, 255, 0), -1); // Turn the right rectangle green
                }
            }
        }
}
