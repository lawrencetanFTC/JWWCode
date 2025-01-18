package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode._0AutonSkeleton.*;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

        Action pushSample1 = drive.actionBuilder(new Pose2d(0, 37, Math.toRadians(-90)))
                .strafeTo(new Vector2d(-31, 37))
                .strafeTo(new Vector2d(-31,10))
                .strafeTo(new Vector2d(-50, 10))
                .strafeTo(new Vector2d(-50,54))
                .strafeTo(new Vector2d(-50,50))
                .build();

        Action PickUpSample



        waitForStart();

        if(isStopRequested()) return;





    }
}
