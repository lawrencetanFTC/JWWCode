package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "ColorBasedMoveWithDistance")
public class PropDetectionProcessor   extends LinearOpMode {

    // Declare motor variables for driving the robot
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    // Declare webcam and pipeline variables for vision processing
    OpenCvCamera webcam;
    ColorDetectionPipeline pipeline;

    // Constants for distance movement calculation using encoders
    static final double COUNTS_PER_MOTOR_REV = 1440;    // Number of encoder counts per motor revolution (TETRIX Motor Encoder)
    static final double WHEEL_DIAMETER_INCHES = 4.0;    // Diameter of the robot's wheels in inches
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * Math.PI);  // Calculate counts per inch traveled

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the motors by mapping them to hardware configuration
        leftMotor = hardwareMap.get(DcMotor.class, "left_drive");
        rightMotor = hardwareMap.get(DcMotor.class, "right_drive");

        // Reset motor encoders before starting
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to run using encoders to track distance
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize the camera and set up the vision pipeline for detecting color
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new ColorDetectionPipeline();  // Set up custom color detection pipeline
        webcam.setPipeline(pipeline);  // Assign the pipeline to the webcam

        // Open the camera synchronously (blocking call)
        webcam.openCameraDevice();

        // Start streaming camera frames at 320x240 resolution and set the camera orientation upright
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        // Wait for the autonomous period to begin
        waitForStart();

        // Main loop for the autonomous operation
        while (opModeIsActive()) {
            // If the color blue is detected, move the robot forward by 3 inches
            if (pipeline.isBlueDetected()) {
                moveRobotByDistance(3);  // Move forward 3 inches at 50% motor power
                telemetry.addData("Color", "Blue Detected - Moving Forward");
            }
            // If the color red is detected, move the robot backward by 3 inches
            else if (pipeline.isRedDetected()) {
                moveRobotByDistance(-3);  // Move backward 3 inches at 50% motor power
                telemetry.addData("Color", "Red Detected - Moving Backward");
            } else {
                // No color detected, so the robot does not move
                telemetry.addData("Color", "No Color Detected");
            }

            // Update telemetry to display the detected color and actions taken
            telemetry.update();
            sleep(100);  // Short delay to prevent excessive updates and allow the robot to stabilize
        }
    }

    // Method to move the robot a specific distance (in inches) based on encoder counts
    private void moveRobotByDistance(double inches) {
        int newLeftTarget;
        int newRightTarget;

        // Calculate the new target position for each motor based on the desired distance
        newLeftTarget = leftMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        newRightTarget = rightMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

        // Set the new target positions for the motors
        leftMotor.setTargetPosition(newLeftTarget);
        rightMotor.setTargetPosition(newRightTarget);

        // Set the motors to RUN_TO_POSITION mode to move to the target position
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor power to start moving the robot
        leftMotor.setPower(Math.abs(0.5));
        rightMotor.setPower(Math.abs(0.5));

        // Continue moving the robot until both motors reach the target position
        while (opModeIsActive() && (leftMotor.isBusy() && rightMotor.isBusy())) {
            // Display telemetry information about the current position and target
            telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            telemetry.addData("Path2", "Running at %7d :%7d", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop the motors once the target position is reached
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        // Set motors back to normal RUN_USING_ENCODER mode
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Custom OpenCV pipeline to detect blue and red colors from the camera feed
    static class ColorDetectionPipeline extends OpenCvPipeline {
        private boolean blueDetected = false;  // Flag to indicate if blue is detected
        private boolean redDetected = false;   // Flag to indicate if red is detected

        // Define HSV color ranges for detecting blue and red colors
        private final Scalar lowBlue = new Scalar(100, 150, 70);   // Lower HSV boundary for blue
        private final Scalar highBlue = new Scalar(140, 255, 255); // Upper HSV boundary for blue
        private final Scalar lowRed = new Scalar(0, 150, 70);      // Lower HSV boundary for red
        private final Scalar highRed = new Scalar(10, 255, 255);   // Upper HSV boundary for red

        @Override
        public Mat processFrame(Mat input) {
            // Convert the camera frame from RGB to HSV color space for color filtering
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Create masks to filter out blue and red colors based on the defined HSV ranges
            Mat blueMask = new Mat();
            Mat redMask = new Mat();
            Core.inRange(hsv, lowBlue, highBlue, blueMask);  // Blue mask for detecting blue areas
            Core.inRange(hsv, lowRed, highRed, redMask);     // Red mask for detecting red areas

            // Calculate the percentage of the frame that is blue and red
            double bluePercentage = Core.sumElems(blueMask).val[0] / blueMask.total() / 255;
            double redPercentage = Core.sumElems(redMask).val[0] / redMask.total() / 255;

            // If more than 10% of the frame is blue or red, mark the color as detected
            blueDetected = bluePercentage > 0.1;
            redDetected = redPercentage > 0.1;

            // Return the unmodified input frame (used for display purposes)
            return input;
        }

        // Method to check if blue is detected
        public boolean isBlueDetected() {
            return blueDetected;
        }

        // Method to check if red is detected
        public boolean isRedDetected() {
            return redDetected;
        }
    }
}
