package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "AutonomousTest2", group = "Autonomous")
public class BlueBasketAutonomous extends LinearOpMode {
    double motorPower = 0.24; // 24%double motorPower = 0.24; // 24%
    
    @Override
    public void runOpMode() {
        // Starting pose (0, 0, 0)
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setPower(motorPower);
        backLeft.setPower(motorPower);
        frontRight.setPower(motorPower);
        backRight.setPower(motorPower);

        waitForStart();

        if (opModeIsActive()) {
            // Move to Waypoint 1 (from (0, 0) to (-6, -24))
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .splineTo(new Vector2d(-6, -24), Math.toRadians(-90)) // Facing down
                            .build()
            );

            // Move to Waypoint 2 (-34, -24)
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(-6, -24, Math.toRadians(-90)))
                            .splineTo(new Vector2d(-34, -24), Math.toRadians(180)) // Move left along X-axis
                            .build()
            );

            // Move to Waypoint 3 (-34, -48)
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(-34, -24, Math.toRadians(180)))
                            .splineTo(new Vector2d(-34, -48), Math.toRadians(-90)) // Move down along Y-axis
                            .build()
            );

            // Move to Waypoint 4 (20, 0)
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(-34, -48, Math.toRadians(-90)))
                            .splineTo(new Vector2d(20, 0), Math.toRadians(0)) // Move right to final position
                            .build()
            );
        }
    }
}
