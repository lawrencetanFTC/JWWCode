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

        Pose2d initialPose = new Pose2d(-24, -60, Math.toRadians(-90));
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

        Action park = drive.actionBuilder(new Pose2d(0,37,Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(0,40))
                .splineToLinearHeading(new Pose2d(-54, 61, Math.toRadians(0)),Math.toRadians(0))
                .build();

        waitForStart();

        if(isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                    new ParallelAction(
                            goToRungs,
                            new SequentialAction(
                                slides.rungPos(),
                                arm.armUp()
                            )
                    ),
                    park

                    ));



    }
}
