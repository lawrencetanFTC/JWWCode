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

        Pose2d initialPose = new Pose2d(-32.5, -63.5, Math.toRadians(90));



        Extend extend = new Extend(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Spintake spintake = new Spintake(hardwareMap);
        Slides slides = new Slides(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        MecanumDrive drive = new  MecanumDrive(hardwareMap, initialPose);

        Action basketPath = drive.actionBuilder(new Pose2d(-32.69, -63.24, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-2.70, -55.05), Math.toRadians(90.00))
                .splineTo(new Vector2d(-48.90, -50.95), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(-48.34, -44.62), Math.toRadians(90.00))
                .splineTo(new Vector2d(-41.45, -46.11), Math.toRadians(230.00))
                .build();

        Action pushBasket = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(-47.04, -11.46), Math.toRadians(-100.00))
                .splineToLinearHeading(new Pose2d(-47.22, -58.21, Math.toRadians(45.00)), Math.toRadians(263.98))
                .splineToLinearHeading(new Pose2d(-58.59, -13.32, Math.toRadians(90.00)), Math.toRadians(180.00))
                .splineToLinearHeading(new Pose2d(-52.63, -52.44, Math.toRadians(45.00)), Math.toRadians(264.81))
                .splineToLinearHeading(new Pose2d(-32.32, 0.09, Math.toRadians(0.00)), Math.toRadians(0.00))
                .build();


        //path test
        Actions.runBlocking(
                new SequentialAction(
                        pushBasket
                )
        );

        if(isStopRequested()) return;











    }


}
