package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "DeckAuto", group = "Autonomous")
public class DeckAuto extends LinearOpMode {
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
                    drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0))) // Deposit specimen
                            .strafeTo(new Vector2d(6, -24))
                            .build()
            );

            // Move to Waypoint 2 (-48, -24)
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(-6, -24, Math.toRadians(0))) // Move to samples
                            .strafeTo(new Vector2d(-48, -24))
                            .build()
            );

            // Move to Waypoint 3 (-48, -6)
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(-48, -24, Math.toRadians(0)))
                            .strafeToLinearHeading(new Vector2d(-48, -6), Math.toRadians(180)) // Push samples in observation zone
                            .build()
            );

            // Move to Waypoint 4 (-12, -60)
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(-48, -6, Math.toRadians(180)))
                            // change this to linetox or smth
                            .strafeToLinearHeading(new Vector2d(-6, -60), Math.toRadians(90)) // Move to landing zone
                            .build()
            );
        }
    }
}
