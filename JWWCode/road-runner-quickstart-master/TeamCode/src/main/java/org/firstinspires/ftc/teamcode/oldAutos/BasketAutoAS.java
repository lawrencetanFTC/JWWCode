package org.firstinspires.ftc.teamcode.oldAutos;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "BasketAutoAS", group = "Autonomous")
public class BasketAutoAS extends LinearOpMode {
    double motorPower = 0.24; // 24% motor power

    @Override
    public void runOpMode() {
        // Initialize motors
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Ensure motors are stopped before the game starts
        stopMotors(frontLeft, backLeft, frontRight, backRight);

        // Starting pose (0, 0, 0)
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        if (opModeIsActive()) {
            // Set motor power after the game starts
            DefaultMotorPower(frontLeft, backLeft, frontRight, backRight);

            // Move to Waypoint 1 (from (0, 0) to (-34, -24))
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .strafeTo(new Vector2d(-34, -24)) // Drop off specimen
                            .build()
            );
            stopMotors(frontLeft, backLeft, frontRight, backRight);
            DefaultMotorPower(frontLeft, backLeft, frontRight, backRight);

            // Move to Waypoint 2 (24, -24)
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(-34, -24, Math.toRadians(0)))
                            .strafeTo(new Vector2d(24, -24)) // Move to samples
                            .build()
            );
            stopMotors(frontLeft, backLeft, frontRight, backRight);
            DefaultMotorPower(frontLeft, backLeft, frontRight, backRight);

            // Move to Waypoint 3 (20, -6)
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(24, -24, Math.toRadians(0)))
                            .strafeToLinearHeading(new Vector2d(20, -6), Math.toRadians(135)) // Move to basket
                            // Place stuff in high basket code will go here
                            .build()
            );
            stopMotors(frontLeft, backLeft, frontRight, backRight);
            DefaultMotorPower(frontLeft, backLeft, frontRight, backRight);

            // Move back to Waypoint 2 (34, -24) for TeleOp
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(20, -6, Math.toRadians(135)))
                            .strafeToLinearHeading(new Vector2d(34, -24), Math.toRadians(0)) // Move to landing zone
                            .build()
            );
            stopMotors(frontLeft, backLeft, frontRight, backRight);
            DefaultMotorPower(frontLeft, backLeft, frontRight, backRight);
        }
    }

    public void DefaultMotorPower(DcMotor frontLeft, DcMotor backLeft, DcMotor frontRight, DcMotor backRight) {
        frontLeft.setPower(motorPower);
        backLeft.setPower(motorPower);
        frontRight.setPower(motorPower);
        backRight.setPower(motorPower);
    }

    // Method to stop all drive motors
    public void stopMotors(DcMotor frontLeft, DcMotor backLeft, DcMotor frontRight, DcMotor backRight) {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }
}
