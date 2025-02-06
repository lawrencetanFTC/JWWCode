package org.firstinspires.ftc.teamcode;



import org.firstinspires.ftc.teamcode.HelloAuto.*;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "BaskettAuto", group = "Autonomous")
public class SamplePathBasket extends LinearOpMode {

    public void runOpMode() {

        Pose2d initialPose = new Pose2d(13.5, 62.5, Math.toRadians(-90));

        Extend extend = new Extend(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Slides slides = new Slides();
        Thread slideThread = new Thread(new SlidesControl(hardwareMap));
        Arm arm = new Arm(hardwareMap);
        BClaw claw = new BClaw(hardwareMap);



        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Action MoveToSubmersible = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(0, 37))
                .build();

        Action PushSamples = drive.actionBuilder(new Pose2d(0, 37, Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(5, 37))
                .splineToConstantHeading(new Vector2d(36.50, 24.00), Math.toRadians(-90.00))
                .splineToConstantHeading(new Vector2d(48.00, 0.00), Math.toRadians(0.00))
                .strafeToConstantHeading(new Vector2d(48, 52.5))
                .splineToConstantHeading(new Vector2d(58, 0), Math.toRadians(0.00))
                .strafeToConstantHeading(new Vector2d(58, 49))
                .build();

        Action BucketCase = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(53.00, 53.00), Math.toRadians(45))
                .strafeToLinearHeading(new Vector2d(35, 35), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(48.00, -0.00, Math.toRadians(-90.00)), Math.toRadians(0.00))
                .strafeToConstantHeading(new Vector2d(48, 52.5))
                .splineToConstantHeading(new Vector2d(58, 0), Math.toRadians(0.00))
                .strafeToConstantHeading(new Vector2d(58, 49))
                .build();

//        boolean AreWeDoingSpecimen = true;
        boolean AreWeTestingPaths = true;

        waitForStart();

        if (isStopRequested()) return;


        if (AreWeTestingPaths) {
            Actions.runBlocking(
                    new SequentialAction(
                            MoveToSubmersible,
                            PushSamples
                    )

            );
        } else {
            Actions.runBlocking(
                    new SequentialAction(
                            new SleepAction(5),
                            new SequentialAction(
                                    MoveToSubmersible
                            )
                    )
            );

            SlideControl.slidePosition = 4000;

            Actions.runBlocking(
                    new SequentialAction(
                            arm.specimenHang()
                    )
            );

            SlideControl.slidePosition = 1000;

            Actions.runBlocking(
                    new SequentialAction(
                            claw.openClaw(),
                            claw.closeClaw(),
                            arm.sample()
                    )
            );

            SlideControl.slidePosition = 8000;

        }
    }
}

//        else {
//            Actions.runBlocking(
//
//            );
//        }
//
//
////        Actions.runBlocking(new SequentialAction(
////                PushSamples
////        ));
//
////        if(isStopRequested());
//
//
//
//
//
//
//
//
//
//
//
//    }
//
//
//}
