package org.firstinspires.ftc.teamcode;



import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.HelloAuto.*;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "hardwareTest", group = "Autonomous")
public class HardwareTest extends LinearOpMode {

    public class EditControl implements Action {
        double newClawTopPosition;
        double newShoulderLeftPosition;
        double newShoulderRightPosition;
        double newClawBottomPosition;
        double newClawWristPosition;
        double newTopPivotPosition;
        double newBottomPivotPosition;
        double newExtendPosition;
        double newWristRightPosition;


        EditControl(double newClawTopPosition, double newShoulderLeftPosition, double newShoulderRightPosition, double newClawBottomPosition, double newClawWristPosition, double newTopPivotPosition, double newBottomPivotPosition, double newExtendPosition, double newWristRightPosition){
            this.newClawTopPosition = newClawTopPosition;
            this.newShoulderLeftPosition = newShoulderLeftPosition;
            this.newShoulderRightPosition = newShoulderRightPosition;
            this.newClawBottomPosition = newClawBottomPosition;
            this.newClawWristPosition = newClawWristPosition;
            this.newTopPivotPosition = newTopPivotPosition;
            this.newBottomPivotPosition = newBottomPivotPosition;
            this.newExtendPosition = newExtendPosition;
            this.newWristRightPosition = newWristRightPosition;
        }


        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            clawServoTop.setPosition(newClawTopPosition);
            shoulderLeft.setPosition(newShoulderLeftPosition);
            shoulderRight.setPosition(newShoulderRightPosition);
            clawServoBottom.setPosition(newClawBottomPosition);
            clawWrist.setPosition(newClawWristPosition);
            topPivot.setPosition(newTopPivotPosition);
            bottomPivot.setPosition(newBottomPivotPosition);
            extendServo.setPosition(newExtendPosition);
            wristRight.setPosition(newWristRightPosition);

            telemetry.addData("CHANGING POS", Math.floor(Math.random() * 10));
            return false;
        }

    }
    public Action updateServo(double newClawTopPosition, double newShoulderLeftPosition, double newShoulderRightPosition, double newClawBottomPosition, double newClawWristPosition, double newTopPivotPosition, double newBottomPivotPosition, double newExtendPosition, double newWristRightPosition) {

        return new EditControl(newClawTopPosition, newShoulderLeftPosition,  newShoulderRightPosition,  newClawBottomPosition,  newClawWristPosition,  newTopPivotPosition, newBottomPivotPosition,  newExtendPosition,  newWristRightPosition);
    }


    private AutonPos ObjectPos;

    Action openClawTop;
    Action closeClawTop;
    Action startPivotTop;
    Action endPivotTop;
    Action sampleGrabWristTop;
    Action specimenGrabWristTop;
    Action sampleScoreWristTop;
    Action specimenScoreWristTop;
    Action startShoulder;
    Action sampleGrabShoulder;
    Action basketShoulder;
    Action specimenGrabShoulder;
    Action specimenScoreShoulder;
    Action retract;
    Action extend;
    Action downWristBottom;
    Action middleWristBottom;
    Action scanWristBottom;
    Action upWristBottom;
    Action openClawBottom;
    Action closeClawBottom;
    Action closeClawTightBottom;
    Action startBottomPivot;
    Action EndBottomPivot;


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

        openClawTop = updateServo(.3511, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition);
        closeClawTop = updateServo(0.0889, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition);
        startPivotTop = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, asc.clawWristPosition, .7328, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition);
        endPivotTop = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, asc.clawWristPosition, .0322, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition);
        sampleGrabWristTop = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, 0.1222, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition);
        specimenGrabWristTop = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, .5478, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition);
        sampleScoreWristTop = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, .6856, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition);
        specimenScoreWristTop = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, .15, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition);
        startShoulder = updateServo(asc.clawTopPosition, .6861, .2661, asc.clawBottomPosition, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition);
        sampleGrabShoulder = updateServo(asc.clawTopPosition, .8467, .1017, asc.clawBottomPosition, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition);
        basketShoulder = updateServo(asc.clawTopPosition, .4111, .5839, asc.clawBottomPosition, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition);
        specimenGrabShoulder = updateServo(asc.clawTopPosition, .8983, .05, asc.clawBottomPosition, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition);
        specimenScoreShoulder = updateServo(asc.clawTopPosition, .1461, .8011, asc.clawBottomPosition, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition);
        retract = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, 0, asc.wristRightPosition);
        extend = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, .56, asc.wristRightPosition);
        downWristBottom = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, .922);
        middleWristBottom = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, .5116);
        scanWristBottom = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, .8206);
        upWristBottom = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, .0978);
        openClawBottom = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, .3817, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition);
        closeClawBottom = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, .18, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition);
        closeClawTightBottom = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, .1122, asc.clawWristPosition, asc.topPivotPosition, asc.bottomPivotPosition, asc.extendPosition, asc.wristRightPosition);
        startBottomPivot = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, asc.clawWristPosition, asc.topPivotPosition, .0367, asc.extendPosition, asc.wristRightPosition);
        EndBottomPivot = updateServo(asc.clawTopPosition, asc.shoulderLeftPosition, asc.shoulderRightPosition, asc.clawBottomPosition, asc.clawWristPosition, asc.topPivotPosition, .4033, asc.extendPosition, asc.wristRightPosition);

        telemetry.addData("BOB ","a");
        telemetry.update();
    }

    public void runOpMode() {

        Pose2d initialPose = new Pose2d(23.5, 62.5, Math.toRadians(-90));


        waitForStart();
        if(isStopRequested()) return;

        change();

        telemetry.addData("I AM DOUBLE DEAD INSIDE", "b");
        telemetry.update();

        Actions.runBlocking(new SleepAction(1));

        telemetry.addData("I AM TRIPLE DEAD INSIDE", "c");
        telemetry.update();

        Actions.runBlocking(
                new SequentialAction(
                        openClawTop, new SleepAction(2),
                        closeClawTop, new SleepAction(2),
                        startPivotTop,new SleepAction(2),
                        endPivotTop,new SleepAction(2),
                        sampleGrabWristTop,new SleepAction(2),
                        specimenGrabWristTop, new SleepAction(2),
                        sampleScoreWristTop, new SleepAction(2),
                        specimenScoreWristTop, new SleepAction(2),
                        startShoulder, new SleepAction(2),
                        sampleGrabShoulder, new SleepAction(2),
                        basketShoulder, new SleepAction(2),
                        specimenGrabShoulder, new SleepAction(2),
                        specimenScoreShoulder, new SleepAction(2),
                        retract, new SleepAction(2),
                        extend, new SleepAction(2),
                        downWristBottom, new SleepAction(2),
                        middleWristBottom, new SleepAction(2),
                        scanWristBottom, new SleepAction(2),
                        upWristBottom, new SleepAction(2),
                        openClawBottom, new SleepAction(2),
                        closeClawBottom, new SleepAction(2),
                        closeClawTightBottom, new SleepAction(2),
                        startBottomPivot, new SleepAction(2),
                        EndBottomPivot
                )


        );
        telemetry.addData("WE ARE QUADRUPLE DEAD INSIDE", "d");
        telemetry.update();

    }}
