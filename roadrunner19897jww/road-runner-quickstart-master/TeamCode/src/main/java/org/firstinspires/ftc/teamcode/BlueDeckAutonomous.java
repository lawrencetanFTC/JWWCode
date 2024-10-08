package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;

@Autonomous(name = "AutonomousTestOne", group = "Autonomous")
public class BlueDeckAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Starting pose (0, 0, 0)
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        if (opModeIsActive()) {
            // Move to Waypoint 1 (from (0, 0) to (-5, -24))
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .splineTo(new Vector2d(-5, -24), Math.toRadians(-90)) // Facing down
                            .build()
            );

            // Move to Waypoint 2 (-48, -24)
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(-5, -24, Math.toRadians(-90)))
                            .splineTo(new Vector2d(-48, -24), Math.toRadians(-90)) // Continue along Y axis
                            .build()
            );

            // Move to Waypoint 3 (-48, -60)
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(-48, -24, Math.toRadians(-90)))
                            .splineTo(new Vector2d(-48, -60), Math.toRadians(-90)) // Move straight down
                            .build()
            );

            // Move to Waypoint 4 (-6, -60)
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(-48, -60, Math.toRadians(-90)))
                            .splineTo(new Vector2d(-6, -60), Math.toRadians(180)) // Move right
                            .build()
            );
        }
    }
}
