package org.firstinspires.ftc.teamcode;



//import org.firstinspires.ftc.teamcode.HelloAuto.*;

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

        Pose2d initialPose = new Pose2d(-32.5, -63, Math.toRadians(-90.00));
        /*
        Extend extend = new Extend(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Slides slides = new Slides();
        Thread slideThread = new Thread(new SlidesControl(hardwareMap));
        Arm arm = new Arm(hardwareMap);
        BClaw claw = new BClaw(hardwareMap);*/
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

         Action NewBasket = drive.actionBuilder(new Pose2d(-32.5, -63, Math.toRadians(-90.00)))
            .strafeToConstantHeading(new Vector2d(-32.5, -57))
            .splineToLinearHeading(new Pose2d(-53.5, -54, Math.toRadians(45.00)), Math.toRadians(-135.00))
            .splineToLinearHeading(new Pose2d(-51, -52, Math.toRadians(90.00)), Math.toRadians(90.00))
            .splineToLinearHeading(new Pose2d(-53.5, -54, Math.toRadians(45.00)), Math.toRadians(-135.00))
            .splineToLinearHeading(new Pose2d(-61, -52, Math.toRadians(90.00)), Math.toRadians(90.00))
            .splineToLinearHeading(new Pose2d(-53.5, -54, Math.toRadians(45.00)), Math.toRadians(-135.00))
            .splineToLinearHeading(new Pose2d(-28, -12, Math.toRadians(180.00)), Math.toRadians(-45.00))
            // We do level 1 ascent here
            .build();

         Action goToBucket = drive.actionBuilder(initialPose)
                 .strafeToConstantHeading(new Vector2d(-32.5, -57))
                 .splineToLinearHeading(new Pose2d(-53.5, -54, Math.toRadians(45.00)), Math.toRadians(-135.00))
                 .build();

         Action goBackToBucket1 = drive.actionBuilder(new Pose2d(-51, -52, Math.toRadians(90.00)))
                 .splineToLinearHeading(new Pose2d(-53.5, -54, Math.toRadians(45.00)), Math.toRadians(-135.00))
                 .build();

        Action goBackToBucket2 = drive.actionBuilder(new Pose2d(-61, -52, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(-53.5, -54, Math.toRadians(45.00)), Math.toRadians(-135.00))
                .build();

        Action sample1 = drive.actionBuilder(new Pose2d(-53.5, -54, Math.toRadians(45.00)))
                .splineToLinearHeading(new Pose2d(-51, -52, Math.toRadians(90.00)), Math.toRadians(90.00))
                .build();

        Action sample2 = drive.actionBuilder(new Pose2d(-53.5, -54, Math.toRadians(45.00)))
                .splineToLinearHeading(new Pose2d(-61, -52, Math.toRadians(90.00)), Math.toRadians(90.00))
                .build();

        Action accent = drive.actionBuilder(new Pose2d(-53.5, -54, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(-28, -12, Math.toRadians(180.00)), Math.toRadians(-45.00))
                .build();

//        boolean AreWeDoingSpecimen = true;
        boolean AreWeTestingPaths = true;

        waitForStart();

        if (isStopRequested()) return;


        if (AreWeTestingPaths) {
            Actions.runBlocking(
                    new SequentialAction(
                            goToBucket,
                            sample1,
                            goBackToBucket1,
                            sample2,
                            goBackToBucket2,
                            accent
                    )

            );
        } else {


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
