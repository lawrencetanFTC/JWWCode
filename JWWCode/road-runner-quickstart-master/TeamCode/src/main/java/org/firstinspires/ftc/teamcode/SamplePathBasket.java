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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "BaskettAuto", group = "Autonomous")
public class SamplePathBasket extends LinearOpMode {

    private AutonPos ObjectPos;

    private DcMotor leftSlideMotor;
    private DcMotor rightSlideMotor;

    // Servos
    private Servo topPivot;
    private Servo bottomPivot;
    private Servo shoulderLeft;
    private Servo shoulderRight;
    private Servo clawServoBottom;
    private Servo clawServoTop;
    private Servo clawWrist;
    private Servo extendServo;
    private Servo wristLeft;
    private Servo wristRight;

    // Slide boundaries
    private final int MAX_POSITION = 8100; // Maximum encoder ticks (full extension)
    private final int MIN_POSITION = 0;

    public void change() {
        leftSlideMotor = hardwareMap.get(DcMotor.class, "leftSlideMotor");
        rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");
        // Reset encoders for the slide motors
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Set them to run using encoders after reset
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shoulderLeft = hardwareMap.get(Servo.class, "shoulderLeft");
        shoulderRight = hardwareMap.get(Servo.class, "shoulderRight");

        extendServo = hardwareMap.get(Servo.class, "extendServo");

        wristRight = hardwareMap.get(Servo.class, "wristRight");
        wristLeft = hardwareMap.get(Servo.class, "wristLeft");

        clawServoBottom = hardwareMap.get(Servo.class, "clawServoBottom");
        clawServoTop = hardwareMap.get(Servo.class, "clawServoTop");

        topPivot = hardwareMap.get(Servo.class, "topPivot");
        bottomPivot = hardwareMap.get(Servo.class, "bottomPivot");

        clawWrist = hardwareMap.get(Servo.class, "clawWrist");
    }

    public void runOpMode() {

        Pose2d initialPose = new Pose2d(23.5, 62.5, Math.toRadians(-90));

        Extend extend = new Extend(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Slides slides = new Slides();
        Thread slideThread = new Thread(new SlidesControl(hardwareMap));
        Arm arm = new Arm(hardwareMap);
        BClaw claw = new BClaw(hardwareMap);



        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Action MoveToSubmersible = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(0, 40))
                .build();

        Action FirstPositionBucket = drive.actionBuilder(new Pose2d(0, 40, Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(5, 40))
                .splineToLinearHeading(new Pose2d(36.50, 24.00, Math.toRadians(90.00)), Math.toRadians(-90)) //
                .splineToConstantHeading(new Vector2d(48.00, 0.00), Math.toRadians(0.00)).build();

        Action FirstOrientBucket = drive.actionBuilder(new Pose2d(48, 0, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(42, 52.5), Math.toRadians(-135))
                .strafeToConstantHeading(new Vector2d(48, 52.5)) //
                .build();

        Action SecondPositionBucket = drive.actionBuilder(new Pose2d(48, 52.5, Math.toRadians(-135)))
                .strafeToConstantHeading(new Vector2d(46.5, 52.5))
                .splineToLinearHeading(new Pose2d(58, 0, Math.toRadians(90.00)), Math.toRadians(0.00))
                .build();

        Action SecondOrientBucket = drive.actionBuilder(new Pose2d(58, 0, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(48, 52.5), Math.toRadians(-135)).build();

        Action PushSamples = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(0, 40))
                .strafeToConstantHeading(new Vector2d(5, 40))
                .splineToConstantHeading(new Vector2d(36.50, 24.00), Math.toRadians(-90.00))
                .splineToConstantHeading(new Vector2d(48.00, 0.00), Math.toRadians(0.00))
                .strafeToConstantHeading(new Vector2d(48, 52.5))
                .splineToConstantHeading(new Vector2d(58, 0), Math.toRadians(0.00))
                .strafeToConstantHeading(new Vector2d(58, 49))
                .build();

        Action BucketCase = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(0, 40))
                .strafeToConstantHeading(new Vector2d(5, 40))
                .splineToLinearHeading(new Pose2d(36.50, 24.00, Math.toRadians(90.00)), Math.toRadians(-90)) //
                .splineToConstantHeading(new Vector2d(48.00, 0.00), Math.toRadians(0.00))
                .strafeToLinearHeading(new Vector2d(42, 52.5), Math.toRadians(-135))
                .strafeToConstantHeading(new Vector2d(48, 52.5)) //
                .strafeToConstantHeading(new Vector2d(46.5, 52.5))
                .splineToLinearHeading(new Pose2d(58, 0, Math.toRadians(90.00)), Math.toRadians(0.00))
                .strafeToLinearHeading(new Vector2d(48, 52.5), Math.toRadians(-135))
                .build();

        Action openClawTop = ObjectPos.openClawTop();
        Action closeClawTop = ObjectPos.closeClawTop();
        Action startPivotTop = ObjectPos.startPivotTop();
        Action endPivotTop = ObjectPos.endPivotTop();
        Action sampleGrabWristTop = ObjectPos.sampleGrabWristTop();
        Action specimenGrabWristTop = ObjectPos.specimenGrabWristTop();
        Action sampleScoreWristTop = ObjectPos.sampleScoreWristTop();
        Action specimenScoreWristTop = ObjectPos.specimenScoreWristTop();
        Action startShoulder = ObjectPos.startShoulder();
        Action sampleGrabShoulder = ObjectPos.sampleGrabShoulder();
        Action basketShoulder = ObjectPos.basketShoulder();
        Action specimenGrabShoulder = ObjectPos.specimenGrabShoulder();
        Action specimenScoreShoulder = ObjectPos.specimenScoreShoulder();
        Action retract = ObjectPos.retract();
        Action extendForm = ObjectPos.extendForm();
        Action downWristBottom = ObjectPos.downWristBottom();
        Action middleWristBottom = ObjectPos.middleWristBottom();
        Action scanWristBottom = ObjectPos.scanWristBottom();
        Action upWristBottom = ObjectPos.upWristBottom();
        Action openClawBottom = ObjectPos.openClawBottom();
        Action closeClawBottom = ObjectPos.closeClawBottom();
        Action tightCloseClawBottom = ObjectPos.tightCloseClawBottom();
        Action startPivotBottom = ObjectPos.startPivotBottom();
        Action endPivotBottom = ObjectPos.endPivotBottom();

        boolean AreWeDoingBucket = true;
//        boolean AreWeTestingPaths = true;

        waitForStart();

        if (isStopRequested()) return;


        if (AreWeDoingBucket) {
            Actions.runBlocking(
                    new SequentialAction(
                            MoveToSubmersible,
                            // new ParalellAction(

                            PushSamples

                            // Between each sample

                            // arm
                            //wrist
                            //slide
                            // reverse them.
                    )

            );
        } else {
            Actions.runBlocking(
                    new SequentialAction(
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
