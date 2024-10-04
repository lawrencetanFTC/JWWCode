package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Move In All Directions")
public class AutonomousTest extends LinearOpMode {

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

package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Red Path Movements")
public class AutonomousTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Road Runner drivetrain
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set starting position (Pose2d is (x, y, heading in radians))
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        // Move forward to point 2
        Trajectory moveToPoint2 = drive.trajectoryBuilder(startPose)
                .forward(20)  // Adjust the distance based on field
                .build();

        // Strafe left to point 3
        Trajectory moveToPoint3 = drive.trajectoryBuilder(moveToPoint2.end())
                .strafeLeft(30)  // Adjust the distance
                .build();

        // Move forward to point 4
        Trajectory moveToPoint4 = drive.trajectoryBuilder(moveToPoint3.end())
                .forward(24)  // Adjust the distance
                .build();

        // Strafe right to point 5
        Trajectory moveToPoint5 = drive.trajectoryBuilder(moveToPoint4.end())
                .strafeRight(24)  // Adjust the distance
                .build();

        // Move forward to point 6
        Trajectory moveToPoint6 = drive.trajectoryBuilder(moveToPoint5.end())
                .forward(24)  // Adjust the distance
                .build();

        // Strafe left to point 7
        Trajectory moveToPoint7 = drive.trajectoryBuilder(moveToPoint6.end())
                .strafeLeft(24)  // Adjust the distance
                .build();

        // Move forward to point 8 (last point)
        Trajectory moveToPoint8 = drive.trajectoryBuilder(moveToPoint7.end())
                .forward(24)  // Adjust the distance
                .build();

        // Wait for the start button to be pressed
        waitForStart();

        if (isStopRequested()) return;

        // Execute all the trajectories in sequence
        drive.followTrajectory(moveToPoint2);
        drive.followTrajectory(moveToPoint3);
        drive.followTrajectory(moveToPoint4);
        drive.followTrajectory(moveToPoint5);
        drive.followTrajectory(moveToPoint6);
        drive.followTrajectory(moveToPoint7);
        drive.followTrajectory(moveToPoint8);

        // End of the autonomous path
    }