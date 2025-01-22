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

@Autonomous(name = "BasketAuto", group = "Autonomous")
public class ActualFinalAuto_Basket extends LinearOpMode {
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(-15, 48 , Math.toRadians(90));



        Extend extend = new Extend(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Spintake spintake = new Spintake(hardwareMap);
        Slides slides = new Slides(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        MecanumDrive drive = new  MecanumDrive(hardwareMap, initialPose);


        Action MoveToBasket = drive.actionBuilder(initialPose)
                .splineToSplineHeading(new Pose2d(52, 52, Math.toRadians(45.00)), Math.toRadians(45.00))
                .build();

        Action MoveToSample1 = drive.actionBuilder(new Pose2d(53,53,Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(48,52), Math.toRadians(-90))
                .build();

        Action Score1 = drive.actionBuilder(new Pose2d(48,51, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(52,52), Math.toRadians(45.00))
                .build();

        Action MoveToSample2 = drive.actionBuilder(new Pose2d(53,53, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(58,45), Math.toRadians(-90))
                .build();

        Action Score2 = drive.actionBuilder(new Pose2d(52,51, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(52,52), Math.toRadians(45.00))
                .build();

        Action MoveToSample3 = drive.actionBuilder(new Pose2d(53,53, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(48, 27), Math.toRadians(0))
                .build();

        Action Score3 = drive.actionBuilder(new Pose2d(47,27, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(53.00, 53.00), Math.toRadians(45))
                .build();

        Action MoveToSubmersible = drive.actionBuilder(new Pose2d(53,53, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(25.00, 5, Math.toRadians(180.00)), Math.toRadians(150.00))
                .build();

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        MoveToBasket,
                            new SequentialAction(
                                    slides.basketPos(),
                                    arm.armUp(),
                                    claw.openClaw()

                            )
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                            slides.lowPos(),
                            arm.armDown()
                        ),

                        new ParallelAction(
                            MoveToSample1,
                            extend.extendSt(),
                            spintake.intake(),
                            wrist.wristDown()

                        )
                )

        );

        Actions.runBlocking(
                new ParallelAction(
                        Score1,
                        spintake.neutral(),
                        extend.retractSt(),
                        wrist.wristUp(),
                        new SequentialAction(
                            new SleepAction(.2),
                            claw.closeClaw(),
                            slides.basketPos(),
                            arm.armUp(),
                            claw.openClaw(),
                            arm.armDown(),
                            slides.lowPos()
                        )


                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        MoveToSample2,
                        extend.extendSt(),
                        wrist.wristDown(),
                        spintake.intake()

                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        Score2,
                        spintake.neutral(),
                        extend.retractSt(),
                        wrist.wristUp(),
                        new SequentialAction(
                                new SleepAction(.2),
                                claw.closeClaw(),
                                slides.basketPos(),
                                arm.armUp(),
                                claw.openClaw(),
                                arm.armDown(),
                                slides.lowPos()
                        )


                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        MoveToSample3,
                        extend.extendSt(),
                        spintake.intake(),
                        wrist.wristDown()

                )
        );


        Actions.runBlocking(
                new ParallelAction(
                        Score3,
                        spintake.neutral(),
                        extend.retractSt(),
                        wrist.wristUp(),
                        new SequentialAction(
                                new SleepAction(.2),
                                claw.closeClaw(),
                                slides.basketPos(),
                                arm.armUp(),
                                claw.openClaw(),
                                arm.armDown(),
                                slides.lowPos()
                        )


                )
        );


        Actions.runBlocking(new SequentialAction(
                MoveToSubmersible,
                slides.rungPos(),
                slides.lowPos()
        ));

        if(isStopRequested()) return;











    }


}
