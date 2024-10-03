package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Move In All Directions")
public class MoveInAllDirections extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Road Runner drivetrain
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set starting position (pose)
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        // Move Forward (e.g., 24 inches)
        Trajectory forwardTrajectory = drive.trajectoryBuilder(startPose)
                .forward(24)
                .build();

        // Move Backward (e.g., 24 inches)
        Trajectory backwardTrajectory = drive.trajectoryBuilder(forwardTrajectory.end())
                .back(24)
                .build();

        // Strafe Left (e.g., 24 inches)
        Trajectory leftTrajectory = drive.trajectoryBuilder(backwardTrajectory.end())
                .strafeLeft(24)
                .build();

        // Strafe Right (e.g., 24 inches)
        Trajectory rightTrajectory = drive.trajectoryBuilder(leftTrajectory.end())
                .strafeRight(24)
                .build();

        // Wait for start button
        waitForStart();

        if (isStopRequested()) return;

        // Execute trajectories
        drive.followTrajectory(forwardTrajectory);
        drive.followTrajectory(backwardTrajectory);
        drive.followTrajectory(leftTrajectory);
        drive.followTrajectory(rightTrajectory);

        // End of autonomous
    }
}
