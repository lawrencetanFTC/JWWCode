package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "BasketAuto", group = "Autonomous")
public class BasketAuto extends LinearOpMode {
    double motorPower = 0.24; // 24%double motorPower = 0.24; // 24%

    @Override
    public void runOpMode() {

        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setPower(motorPower);
        backLeft.setPower(motorPower);
        frontRight.setPower(motorPower);
        backRight.setPower(motorPower);
        // Starting pose (0, 0, 0)
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        if (opModeIsActive()) {
            // Move to Waypoint 1 (from (0, 0) to (-34, -24))
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .strafeTo(new Vector2d(-34, -24)) // Drop off specimen
                            .build()
            );

            // Move to Waypoint 2 (24, -24)
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(-34, -24, Math.toRadians(0)))
                            .strafeTo(new Vector2d(24, -24)) // Move to samples
                            .build()
            );

            // Move to Waypoint 3 (20, -6)
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(24, -24, Math.toRadians(0)))
                            .strafeToLinearHeading(new Vector2d(20, -6), Math.toRadians(135)) // Move to basket
                            // Palce stuff in high basket code will go here
                            .build()
            );

            // Move to Waypoint 4 (-14, -60)
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(20, -6, Math.toRadians(135)))
                            .strafeToLinearHeading(new Vector2d(-14, -60), Math.toRadians(-90)) // Move to landing zone
                            .build()
            );
        }
    }
}
