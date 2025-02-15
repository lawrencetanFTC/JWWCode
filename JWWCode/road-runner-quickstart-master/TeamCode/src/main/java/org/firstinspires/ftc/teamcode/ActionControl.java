package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ActionControl {

    public boolean runThread = true;
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

    ActionControl(@NonNull HardwareMap hardwareMap) {
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

        clawServoBottom = hardwareMap.get(Servo.class, "clawServoBottom");
        clawServoTop = hardwareMap.get(Servo.class, "clawServoTop");

        topPivot = hardwareMap.get(Servo.class, "topPivot");
        bottomPivot = hardwareMap.get(Servo.class, "bottomPivot");

        clawWrist = hardwareMap.get(Servo.class, "clawWrist");
    }

    public void resetToInitial() {
        clawServoTop.setPosition(clawTopPosition);
        shoulderLeft.setPosition(shoulderLeftPosition);
        shoulderRight.setPosition(shoulderRightPosition);
        clawServoBottom.setPosition(clawBottomPosition);
        clawWrist.setPosition(clawWristPosition);
        topPivot.setPosition(topPivotPosition);
        bottomPivot.setPosition(bottomPivotPosition);
        extendServo.setPosition(extendPosition);
        wristRight.setPosition(wristRightPosition);
    }


    private TelemetryPacket telemetryPacket;

//    public Action updateServo(double newClawTopPosition, double newShoulderLeftPosition, double newShoulderRightPosition, double newClawBottomPosition, double newClawWristPosition, double newTopPivotPosition, double newBottomPivotPosition, double newExtendPosition, double newWristRightPosition) {
//        return Actions.runBlocking(() -> {
//            clawServoTop.setPosition(newClawTopPosition);
//            shoulderLeft.setPosition(newShoulderLeftPosition);
//            shoulderRight.setPosition(newShoulderRightPosition);
//            clawServoBottom.setPosition(newClawBottomPosition);
//            clawWrist.setPosition(newClawWristPosition);
//            topPivot.setPosition(newTopPivotPosition);
//            bottomPivot.setPosition(newBottomPivotPosition);
//            extendServo.setPosition(newExtendPosition);
//            wristRight.setPosition(newWristRightPosition);
//
//            return null; // Explicitly return null or an empty Action if required
//        });
//    }

    class ThreadedControl implements Runnable {

        public void run() {
            while (runThread) {
                clawServoTop.setPosition(clawTopPosition);
                shoulderLeft.setPosition(shoulderLeftPosition);
                shoulderRight.setPosition(shoulderRightPosition);
                clawServoBottom.setPosition(clawBottomPosition);
                clawWrist.setPosition(clawWristPosition);
                topPivot.setPosition(topPivotPosition);
                bottomPivot.setPosition(bottomPivotPosition);
                extendServo.setPosition(extendPosition);
                wristRight.setPosition(wristRightPosition);
            }
        }
    }

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
}

