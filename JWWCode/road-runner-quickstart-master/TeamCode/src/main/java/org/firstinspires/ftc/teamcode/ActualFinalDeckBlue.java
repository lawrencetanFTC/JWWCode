package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode._0AutonSkeleton.Arm;
import org.firstinspires.ftc.teamcode._0AutonSkeleton.Claw;
import org.firstinspires.ftc.teamcode._0AutonSkeleton.Extend;
import org.firstinspires.ftc.teamcode._0AutonSkeleton.Slides;
import org.firstinspires.ftc.teamcode._0AutonSkeleton.Spintake;
import org.firstinspires.ftc.teamcode._0AutonSkeleton.Wrist;

@Autonomous
public class ActualFinalDeckBlue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(-33, 53, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Extend extend = new Extend(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Spintake spintake = new Spintake(hardwareMap);
        Slides slides = new Slides(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        Action goToRungs = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(0, 37))
                .build();

        Action pushSamples = drive.actionBuilder(new Pose2d(0, 37, Math.toRadians(-90)))
                .strafeTo(new Vector2d(-31, 37))
                .turn(Math.toRadians(90))
                .strafeTo(new Vector2d(-31,10))
                .strafeTo(new Vector2d(-41, 10))
                .strafeTo(new Vector2d(-41,54))
                .strafeTo(new Vector2d(-41,10))
                .strafeTo(new Vector2d(-53,10))
                .strafeTo(new Vector2d(-53, 54))
                .build();


        Action ScoreSpecimen = drive.actionBuilder(new Pose2d(-53,54, Math.toRadians(90)))
                .strafeTo(new Vector2d(-53,45))
                .splineTo(new Vector2d(-40.09, 46.26), Math.toRadians(-30.94))
                .splineToLinearHeading(new Pose2d(-23.00, 59.03, Math.toRadians(180.00)), Math.toRadians(180.00))
                .splineTo(new Vector2d(-41.17, 47.09), Math.toRadians(7.31))
                .splineTo(new Vector2d(-30.19, 46.87), Math.toRadians(2.10))
                .splineTo(new Vector2d(-1.69, 47.51), Math.toRadians(45.00))
                .splineToLinearHeading(new Pose2d(0.00, 37.00, Math.toRadians(-90.00)), Math.toRadians(-90.00))
                .build();





        waitForStart();

        if(isStopRequested()) return;





    }
}
