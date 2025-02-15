// All code is written by me except when mentioned otherwise.
//Copied that same way it is written in the rr.brott.dev website.
// Then you can take these methods and use followTrajectoryAsync(traj1), this will allow for simultaneous robot movements along with drivetrain.

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.transition.Slide;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "LastAutoDeck", group = "Autonomous")
// @Disabled
public class LastAutoDeck extends LinearOpMode {

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
        return new LastAutoDeck.EditControl();
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

    // Slide boundaries
    private final int MAX_POSITION = 8100; // Maximum encoder ticks (full extension)
    private final int MIN_POSITION = 0;

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

    }

    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(15,-63.5, Math.toRadians(90.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // init all the non drive train stuff here
        // int increment = 5;

                // .strafeToConstantHeading(new Vector2d(0, -47)) // Hang Specimen
                // .strafeToConstantHeading(new Vector2d(10, -47))
                // // .splineToConstantHeading(new Vector2d(36.50, -24.00), Math.toRadians(90.00))
                // .splineToConstantHeading(new Vector2d(48.00, 0.00), Math.toRadians(90.00))
                // .strafeToConstantHeading(new Vector2d(48, -52.5))
                // .splineToConstantHeading(new Vector2d(58, 0), Math.toRadians(90.00))
                // .strafeToConstantHeading(new Vector2d(58, -49))
                // .turn(-180)
                // .strafeToLinearHeading(new Vector2d(0, -47), Math.toRadians(90))
                // .strafeToLinearHeading(new Vector2d(58, -49), Math.toRadians(-90))
                // .strafeToLinearHeading(new Vector2d(0, -47), Math.toRadians(90))
                // .strafeToLinearHeading(new Vector2d(58, -49), Math.toRadians(-90))
                // .strafeToLinearHeading(new Vector2d(0, -47), Math.toRadians(90))
                // .build();



          Action MoveToChamber = drive.actionBuilder(initialPose)
                    .strafeToLinearHeading(new Vector2d(0,-47), Math.toRadians(-90))
                    .build();

            
          Action PushSamples = drive.actionBuilder(new Pose2d(0, -47, Math.toRadians(90)))
                    .strafeToConstantHeading(new Vector2d(10, -47))
                    // .splineToConstantHeading(new Vector2d(36.50, -24.00), Math.toRadians(90.00))
                    .splineToConstantHeading(new Vector2d(40.00, 0.00), Math.toRadians(90.00))
                    .strafeToLinearHeading(new Vector2d(48, 0), Math.toRadians(-90))
                    .strafeToConstantHeading(new Vector2d(48, -52.5))
                    .splineToConstantHeading(new Vector2d(58, 0), Math.toRadians(0.00))
                    .strafeToConstantHeading(new Vector2d(58, -49))// Move
                    .build();
    
            Action ClawFirstSpec = drive.actionBuilder(new Pose2d(58, -49, Math.toRadians(90)))
                    .turn(Math.toRadians(-180))
                    .build();

            Action HangSpec = drive.actionBuilder(new Pose2d(58, -49, Math.toRadians(-90)))
                    .strafeToConstantHeading(new Vector2d(0, -47)) // Increment move
                    .build();
    
            Action ClawSpec = drive.actionBuilder(new Pose2d(0, -47, Math.toRadians(90)))
                    .strafeToConstantHeading(new Vector2d(58, -49)) // Increment move
                    .build();
    
            Action SafeStrafe = drive.actionBuilder(new Pose2d(0, - 47, Math.toRadians(90)))
                    .strafeToConstantHeading(new Vector2d(-5, -47))
                    .build();



        // Action GrabSpecFirstTime = drive.actionBuilder(new Pose2d((24 + (3*increment)), (-47 + (3*increment)), Math.toRadians(-30)))
        //         .strafeToLinearHeading(new Vector2d(55,-55), Math.toRadians(-90))
        //         .strafeToLinearHeading(new Vector2d(0, -47), Math.toRadians(90))
        //         .build();
        // Action GrabSpec = drive.actionBuilder(new Pose2d(0,-47, Math.toRadians(90)))
        //         .strafeToLinearHeading(new Vector2d(55,-55), Math.toRadians(-90))
        //         .build();
        // Action Score = drive.actionBuilder(new Pose2d(55,-55,Math.toRadians(-90)))
        //         .strafeToLinearHeading(new Vector2d(0, -47), Math.toRadians(90))
        //         .build();

        if (isStopRequested()) return;
        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                    new SequentialAction(
                        MoveToChamber
                        // ArmPosition
                        // WristPosition
                    ),
                    PushSamples,
                    new SequentialAction(
                        ClawFirstSpec
                        //WristPosition
                    ),
                    new SequentialAction(
                        HangSpec
                        // ArmPosition
                        // WristPostion
                    )
                    // for(int i = 0; i < 2; i++) {
                    // new SequentialAction(
                    //     new SequentialAction(
                    //         AutonSequence.ClawSpec
                    //         // WristPosition
                    //     )
                    //     new SequentialAction(
                    //         AutonSequence.HangSpec
                    //         // ArmPosition
                    //         AutonSequence.SafeStrafe
                    //         // WristPostion
                    //     )
                    // }
                )
        ); // Reverse all positions after each Action :)
        for(int i = 0; i < 2; i++) {
            Actions.runBlocking(
                new SequentialAction(
                        new SequentialAction(
                            ClawSpec
                            // WristPosition
                        ),
                        new SequentialAction(
                            HangSpec,
                            // ArmPosition
                            SafeStrafe
                            // WristPosition
                        )
                )
            );
        }
    }
}
