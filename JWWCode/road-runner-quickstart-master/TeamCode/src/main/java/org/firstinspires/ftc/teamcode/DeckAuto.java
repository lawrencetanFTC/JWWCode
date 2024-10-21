package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "DeckAuto", group = "Autonomous")
public class DeckAuto extends LinearOpMode {
    double motorPower = 0.24; // 24% motor power

    @Override
    public void runOpMode() {
        // Starting pose (0, 0, 0)
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        // Initialize motors
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Ensure motors are stopped before the game starts
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        waitForStart();

        if (opModeIsActive()) {
            // Set motor power after the game starts
            frontLeft.setPower(motorPower);
            backLeft.setPower(motorPower);
            frontRight.setPower(motorPower);
            backRight.setPower(motorPower);

            // Move to Waypoint 1 (from (0, 0) to (-6, -24))
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0))) // Deposit specimen
                            .strafeTo(new Vector2d(6, -24))
                            .build()
            );
            stopMotors(frontLeft, backLeft, frontRight, backRight);

            // Repeat 3 times {
            // Move to Waypoint 2 (-48, -24)
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(-6, -24, Math.toRadians(0))) // Move to samples
                            .strafeTo(new Vector2d(-48, -24))
                            .build()
            );
            stopMotors(frontLeft, backLeft, frontRight, backRight);

            // Move to Waypoint 3 (-48, -6)
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(-48, -24, Math.toRadians(0)))
                            .strafeToLinearHeading(new Vector2d(-48, -6), Math.toRadians(180)) // Push samples in observation zone
                            .build()
            );
            stopMotors(frontLeft, backLeft, frontRight, backRight);
            // }

            // Move back to Waypoint 2 (6, -24) for TeleOp
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(-48, -6, Math.toRadians(180)))
                            .strafeToLinearHeading(new Vector2d(6, -24), Math.toRadians(0)) // Move to landing zone
                            .build()
            );
            stopMotors(frontLeft, backLeft, frontRight, backRight);
        }
    }

    // Method to stop all drive motors
    public void stopMotors(DcMotor frontLeft, DcMotor backLeft, DcMotor frontRight, DcMotor backRight) {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }
}
