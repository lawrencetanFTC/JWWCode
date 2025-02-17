package org.firstinspires.ftc.teamcode;



import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.HelloAuto.*;
import org.firstinspires.ftc.teamcode.ActionControl;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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

    public class EditControl implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return false;
        }
    }
    public Action updateServo(double newClawTopPosition, double newShoulderLeftPosition, double newShoulderRightPosition, double newClawBottomPosition, double newClawWristPosition, double newTopPivotPosition, double newBottomPivotPosition, double newExtendPosition, double newWristRightPosition) {
        clawServoTop.setPosition(newClawTopPosition);
        shoulderLeft.setPosition(newShoulderLeftPosition);
        shoulderRight.setPosition(newShoulderRightPosition);
        clawServoBottom.setPosition(newClawBottomPosition);
        clawWrist.setPosition(newClawWristPosition);
        topPivot.setPosition(newTopPivotPosition);
        bottomPivot.setPosition(newBottomPivotPosition);
        extendServo.setPosition(newExtendPosition);
        wristRight.setPosition(newWristRightPosition);
        return new EditControl();
    }


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
    private Servo wristRight;

    double kp = 0.01;
    // Integral gain
    double ki = 0.0;
    // Derivative gain
    double kd = 0.001;

    private double targetPosition = 0; // Desired encoder position
    private double integralSum = 0;
    private double lastError = 0;
    private double currentPosition = 0;
    private double power = 0;

    // Constants for slides
    private final int BASKET_SLIDE_POSITION = 3000;     // Maximum encoder ticks (full extension)
    private final int SPECIMEN_HANG_POS = 490;
    private final int SPECIMEN_GRAB_POS = 0;
    private final int MIN_POSITION = 0;       // Minimum encoder ticks (fully retracted)
    private final int NORMAL_INCREMENT = 50;   // Encoder ticks per loop for continuous motion

    // Slide boundaries
    private final int MAX_POSITION = 8100; // Maximum encoder ticks (full extension)

    public double clawTopPosition = 0.0889;
    public double shoulderLeftPosition = 0.6861;
    public double shoulderRightPosition = 0.2661;
    public double clawBottomPosition = 0.18;
    public double clawWristPosition = 0.1222;
    public double topPivotPosition = 0.7328;
    public double bottomPivotPosition = 0.0367;
    public double extendPosition = 0;
    public double wristRightPosition = 0.0978;

    public void change() {
        leftSlideMotor = hardwareMap.get(DcMotor.class, "leftSlideMotor");
        rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");
        // Reset encoders for the slide motors
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ActionControl asc = new ActionControl(hardwareMap);
        // Set them to run using encoders after reset
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shoulderLeft = hardwareMap.get(Servo.class, "shoulderLeft");
        shoulderRight = hardwareMap.get(Servo.class, "shoulderRight");

        extendServo = hardwareMap.get(Servo.class, "extendServo");

        wristRight = hardwareMap.get(Servo.class, "wristRight");

        clawServoBottom = hardwareMap.get(Servo.class, "clawServoBottom");
        clawServoTop = hardwareMap.get(Servo.class, "clawServoTop");

        topPivot = hardwareMap.get(Servo.class, "topPivot");
        bottomPivot = hardwareMap.get(Servo.class, "bottomPivot");

        clawWrist = hardwareMap.get(Servo.class, "clawWrist");
    }

    public void runOpMode() {

        Pose2d initialPose = new Pose2d(23.5, 62.5, Math.toRadians(-90));

        ActionControl asc = new ActionControl(hardwareMap);

        Extend extendForm = new Extend(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Slides slides = new Slides();
        Thread slideThread = new Thread(new SlidesControl(hardwareMap));
        Arm arm = new Arm(hardwareMap);
        BClaw claw = new BClaw(hardwareMap);

        targetPosition = Math.max(MIN_POSITION, Math.min(BASKET_SLIDE_POSITION, targetPosition));

        // Get the current position from encoders
        double currentPosition = (leftSlideMotor.getCurrentPosition() + rightSlideMotor.getCurrentPosition()) / 2.0;

        // Calculate PID terms
        double error = targetPosition - currentPosition;
        integralSum += error; // Accumulate error for integral term
        double derivative = error - lastError; // Change in error for derivative term

        // Compute motor power using PID
        double power = (kp * error) + (ki * integralSum) + (kd * derivative);

        // Apply power to both motors
        leftSlideMotor.setPower(power);
        rightSlideMotor.setPower(power);

        // Update last error
        lastError = error;



        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Action openClawTop = updateServo(.3511, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition );
        Action closeClawTop = updateServo(0.0889, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition );
        Action startPivotTop = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, asc.clawWristPosition, .7328, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition );
        Action endPivotTop = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, asc.clawWristPosition, .0322, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition );
        Action sampleGrabWristTop = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, 0.1222, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition );
        Action specimenGrabWristTop = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, .5478, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition );
        Action sampleScoreWristTop = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, .6856, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition );
        Action specimenScoreWristTop = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, .15, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition );
        Action startShoulder = updateServo(asc.clawTopPosition, .6861, .2661, asc.clawBottomPosition, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition );
        Action sampleGrabShoulder = updateServo(asc.clawTopPosition, .8467, .1017, asc.clawBottomPosition, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition );
        Action basketShoulder = updateServo(asc.clawTopPosition, .4111, .5839, asc.clawBottomPosition, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition );
        Action specimenGrabShoulder = updateServo(asc.clawTopPosition, .8983, .05, asc.clawBottomPosition, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition );
        Action specimenScoreShoulder = updateServo(asc.clawTopPosition, .1461, .8011, asc.clawBottomPosition, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition );
        Action retract = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, 0, asc.wristRightPosition );
        Action extend = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, .56, asc.wristRightPosition );
        Action downWristBottom = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, .922 );
        Action middleWristBottom = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, .5116 );
        Action scanWristBottom = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, .8206);
        Action upWristBottom = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, .0978 );
        Action openClawBottom = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, .3817, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition );
        Action closeClawBottom = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, .18, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition );
        Action closeClawTightBottom = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, .1122, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition );
        Action startBottomPivot = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, asc.clawWristPosition, asc.topPivotPosition, .0367, asc.extendPosition, asc.wristRightPosition );
        Action EndBottomPivot = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, asc.clawWristPosition, asc.topPivotPosition, .4033, asc.extendPosition, asc.wristRightPosition );

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

        Action PushSamples = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(0, 40))
                .strafeToConstantHeading(new Vector2d(5, 40))
                .splineToConstantHeading(new Vector2d(36.50, 24.00), Math.toRadians(-90.00))
                .splineToConstantHeading(new Vector2d(48.00, 0.00), Math.toRadians(0.00))
                .strafeToConstantHeading(new Vector2d(48, 52.5))
                .splineToConstantHeading(new Vector2d(58, 0), Math.toRadians(0.00))
                .strafeToConstantHeading(new Vector2d(58, 49))
                .build();

//        Action BucketCase = drive.actionBuilder(initialPose)
//                .strafeToConstantHeading(new Vector2d(0, 40))
//                .strafeToConstantHeading(new Vector2d(5, 40))
//                .splineToLinearHeading(new Pose2d(36.50, 24.00, Math.toRadians(90.00)), Math.toRadians(-90)) //
//                .splineToConstantHeading(new Vector2d(48.00, 0.00), Math.toRadians(0.00))
//                .strafeToLinearHeading(new Vector2d(42, 52.5), Math.toRadians(-135))
//                .strafeToConstantHeading(new Vector2d(48, 52.5)) //
//                .strafeToConstantHeading(new Vector2d(46.5, 52.5))
//                .splineToLinearHeading(new Pose2d(58, 0, Math.toRadians(90.00)), Math.toRadians(0.00))
//                .strafeToLinearHeading(new Vector2d(48, 52.5), Math.toRadians(-135))
//                .build();

        boolean AreWeDoingBucket = true;
//        boolean AreWeTestingPaths = true;

        waitForStart();

        if (isStopRequested()) return;


        if (AreWeDoingBucket) {
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    basketShoulder,
                                    specimenScoreWristTop
                            ),
                            endPivotTop
                    )
            );

            targetPosition = 3000;

        } else {
            Actions.runBlocking(
                    new SequentialAction(
                            PushSamples
                    )
            );
        }
    }
}
