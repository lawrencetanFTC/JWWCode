package org.firstinspires.ftc.teamcode;



import org.firstinspires.ftc.teamcode.HelloAuto.*;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;
import org.opencv.core.Mat;

import com.qualcomm.robotcore.hardware.IMU;

@Autonomous
public class ActualFinalBasketBlue extends LinearOpMode {
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
                .splineTo(new Vector2d(53.00, 53.00), Math.toRadians(45))
                .build();

        Action MoveToSample1 = drive.actionBuilder(new Pose2d(53,53,Math.toRadians(45)))
                .turn(Math.toRadians(-135))
                .strafeTo(new Vector2d(48, 51))
                .build();

        Action PickUpandScore1 = drive.actionBuilder(new Pose2d(48,51, Math.toRadians(-90)))
                .strafeTo(new Vector2d(48, 48))
                .splineTo(new Vector2d(53.00, 53.00), Math.toRadians(45))
                .build();

        Action MoveToSample2 = drive.actionBuilder(new Pose2d(53,53, Math.toRadians(45)))
                .turn(-135)
                .strafeTo(new Vector2d(58,51))
                .build();

        Action PickUpandScore2 = drive.actionBuilder(new Pose2d(52,51, Math.toRadians(-90)))
                .strafeTo(new Vector2d(58, 48))
                .splineTo(new Vector2d(53.00, 53.00), Math.toRadians(45))
                .build();

        Action MoveToSample3 = drive.actionBuilder(new Pose2d(53,53, Math.toRadians(45)))
                .turn(-135)
                .splineToLinearHeading(new Pose2d(47,27, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Action PickUpandScore3 = drive.actionBuilder(new Pose2d(47,27, Math.toRadians(0)))
                .strafeTo(new Vector2d(48, 27))
                .splineTo(new Vector2d(53.00, 53.00), Math.toRadians(45))
                .build();

        Action MoveToSubmersible = drive.actionBuilder(new Pose2d(53,53, Math.toRadians(45)))
                .splineTo(new Vector2d(52.00, 45.00), Math.toRadians(238.03))
                .splineToLinearHeading(new Pose2d(25.00, 13.00, Math.toRadians(180.00)), Math.toRadians(180.00))
                .build();

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        MoveToBasket,
                            new SequentialAction(
                                    slides.basketPos(),
                                    arm.armUp()
                            )
                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                            slides.lowPos(),
                            arm.armDown()
                        ),
                        MoveToSample1,
                        extend.extendSt(),
                        spintake.intake()
                )

        );

        Actions.runBlocking(
                new ParallelAction(
                        PickUpandScore1,
                        spintake.neutral(),
                        extend.retractSt(),
                        new SequentialAction(
                            claw.openClaw(),
                            claw.closeClaw(),
                            slides.basketPos(),
                            arm.armUp(),
                            claw.openClaw(),
                            claw.closeClaw(),
                            arm.armDown(),
                            slides.lowPos()
                        )


                )

        );











    }


}
