package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Autonomous")
public class AutonomousTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Road Runner drivetrain
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        // Set starting pose at (0, 0) with a heading of 0 radians
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        // Move to (0, 36) using a spline for a smooth transition
        Trajectory splineToY36 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(0, 36), 0)  // Move along Y-axis, keep heading 0
                .build();

        // Strafe left to (-24, 36) using a linear move
        Trajectory strafeToXNeg24 = drive.trajectoryBuilder(splineToY36.end())
                .lineTo(new Vector2d(-24, 36))  // Strafe left to X = -24
                .build();

        // Spline to (-24, 72) for smooth forward movement
        Trajectory splineToY72 = drive.trajectoryBuilder(strafeToXNeg24.end())
                .splineTo(new Vector2d(-24, 72), 0)  // Move forward along Y-axis, heading unchanged
                .build();

        // Spline to (0, 72) for smooth sideways movement
        Trajectory splineToX0 = drive.trajectoryBuilder(splineToY72.end())
                .splineTo(new Vector2d(0, 72), 0)  // Strafe back to X = 0
                .build();

        // Spline to (0, 96) for smooth forward movement
        Trajectory splineToY96 = drive.trajectoryBuilder(splineToX0.end())
                .splineTo(new Vector2d(0, 96), 0)  // Move forward to Y = 96
                .build();

        // Strafe left to (-24, 96) using a linear move
        Trajectory strafeToXNeg24_2 = drive.trajectoryBuilder(splineToY96.end())
                .lineTo(new Vector2d(-24, 96))  // Strafe left to X = -24
                .build();

        // Spline forward to (-24, 120) for the final smooth movement
        Trajectory splineToY120 = drive.trajectoryBuilder(strafeToXNeg24_2.end())
                .splineTo(new Vector2d(-24, 120), 0)  // Move forward to Y = 120
                .build();

        // Wait for the start button to be pressed
        waitForStart();

        if (isStopRequested()) return;

        // Execute all the trajectories in sequence
        drive.followTrajectory(splineToY36);
        drive.followTrajectory(strafeToXNeg24);
        drive.followTrajectory(splineToY72);
        drive.followTrajectory(splineToX0);
        drive.followTrajectory(splineToY96);
        drive.followTrajectory(strafeToXNeg24_2);
        drive.followTrajectory(splineToY120);

        // End of the autonomous path
    }
}
