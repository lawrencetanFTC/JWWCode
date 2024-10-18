package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;

import java.util.ArrayList;

@Autonomous(name="AprilTag Auto Drive", group="Autonomous")
public class AprilTagsAutonomousTest extends LinearOpMode {

    private DcMotor leftDrive, rightDrive;

    // Tag parameters (Adjust based on tag size and camera)
    private final double TAG_SIZE = 0.166; // meters
    private final double FX = 578.272; // lens intrinsics
    private final double FY = 578.272;
    private final double CX = 402.145;
    private final double CY = 221.506;

    private final double DRIVE_SPEED = 0.2; // driving speed towards the tag
    private final double DISTANCE_TOLERANCE = 0.1; // acceptable distance from tag (meters)

    private AprilTagDetectorJNI detector;
    private long detectorPtr;

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware mapping
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Initialize AprilTag detector
        detector = new AprilTagDetectorJNI();
        detectorPtr = detector.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 0.5f, 4);

        // Initialize camera (you may need to adjust this part based on your camera setup)
        VideoCapture camera = new VideoCapture(0);  // Assumes default camera
        if (!camera.isOpened()) {
            telemetry.addData("Camera Error", "Unable to open camera!");
            telemetry.update();
            return;
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Main autonomous loop
        while (opModeIsActive()) {
            // Capture a frame from the camera
            Mat frame = new Mat();
            camera.read(frame);

            // Detect AprilTags in the frame
            ArrayList<AprilTagDetection> detections = detector.runAprilTagDetectorSimple(detectorPtr, frame, TAG_SIZE, FX, FY, CX, CY);

            if (!detections.isEmpty()) {
                // Process the first detected tag (you can modify to process multiple if needed)
                AprilTagDetection tag = detections.get(0);

                // Retrieve pose data (X, Y, Z distances)
                double x = tag.pose.x;
                double z = tag.pose.z;

                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("X distance", x);
                telemetry.addData("Z distance", z);
                telemetry.update();

                // Move towards the tag based on Z distance (forward/backward movement)
                if (z > DISTANCE_TOLERANCE) {
                    // Move forward
                    leftDrive.setPower(DRIVE_SPEED);
                    rightDrive.setPower(DRIVE_SPEED);
                } else if (z < -DISTANCE_TOLERANCE) {
                    // Move backward
                    leftDrive.setPower(-DRIVE_SPEED);
                    rightDrive.setPower(-DRIVE_SPEED);
                } else {
                    // Stop if within tolerance
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    telemetry.addData("Status", "Reached target");
                    telemetry.update();
                }
            } else {
                telemetry.addData("Status", "No AprilTag detected");
                telemetry.update();
            }

            sleep(50); // Small delay to prevent overloading the loop
        }

        // Release detector resources after operation is complete
        detector.releaseApriltagDetector(detectorPtr);
    }
}
