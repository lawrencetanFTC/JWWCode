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

        Pose2d initialPose = new Pose2d(33, 61.5, Math.toRadians(-90));



        Extend extend = new Extend(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Spintake spintake = new Spintake(hardwareMap);
        Slides slides = new Slides(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        MecanumDrive drive = new  MecanumDrive(hardwareMap, initialPose);


        Action MoveToSubmersible = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(0, 47))

                .build();
        Action PushSamples = drive.actionBuilder(new Pose2d(0,47,Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(5, 47))
                .splineToConstantHeading(new Vector2d(36.50, 24.00), Math.toRadians(-90.00))
                .splineToConstantHeading(new Vector2d(48.00, 0.00), Math.toRadians(0.00))
                .strafeToConstantHeading(new Vector2d(48,52.5))
                .splineToConstantHeading(new Vector2d(58,0), Math.toRadians(0.00))
                .strafeToConstantHeading(new Vector2d(58,49))
                .build();







        waitForStart();





        if(isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        new SleepAction(5),
                new ParallelAction(
                        MoveToSubmersible,
                        slides.rungPos(),
                        arm.armUp()
                ))
        );

        Actions.runBlocking(
                new SequentialAction(
                        slides.hook(),
                        claw.openClaw(),
                        claw.closeClaw(),
                        arm.armDown(),
                        slides.lowPos(),
                        PushSamples
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
