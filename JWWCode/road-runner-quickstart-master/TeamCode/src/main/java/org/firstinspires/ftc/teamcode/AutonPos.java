package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class AutonPos extends OpMode {
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

    @Override
    public void init() {
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

  
    public void openClawTop() {
        clawServoTop.setPosition(.3511);
    }
    
    public void closeClawTop() {
        clawServoTop.setPosition(0.0889);
    }
    
    public void startPivotTop () {
        topPivot.setPosition(.7328);
    }
    
    public void endPivotTop() {
        topPivot.setPosition(.0322);
    }
    
    public void sampleGrabWristTop() {
        clawWrist.setPosition(.0744);
    }
    
    public void specimenGrabWristTop() {
        clawWrist.setPosition(.4883);
    }
    
    public void sampleScoreWristTop() {
        clawWrist.setPosition(.6856);
    }
    
    public void specimenScoreWristTop () {
        clawWrist.setPosition(.15);
    }
    
    public void startShoulder() {
        shoulderLeft.setPosition(.6861);
        shoulderRight.setPosition(.2661);
    }
    
    public void sampleGrabShoulder() {
        shoulderLeft.setPosition(.8828);
        shoulderRight.setPosition(.0717);
    }
    
    public void basketShoulder() {
        shoulderLeft.setPosition(.4111);
        shoulderRight.setPosition(.5839);
    }
    
    public void specimenGrabShoulder() {
        shoulderLeft.setPosition(.8983);
        shoulderRight.setPosition(.05);
    }
    
    public void specimenScoreShoulder() {
        shoulderLeft.setPosition(.1461);
        shoulderRight.setPosition(.8011);
    }
    
    public void retract() {
        extendServo.setPosition(0);
    }
    
    public void extend() {
        extendServo.setPosition(.56);
    }
    
    public void downWristBottom() {
        wristRight.setPosition(.9222);
    }
    
    public void middleWristBottom() {
        wristRight.setPosition(.5116);
    }
    
    public void scanWristBottom() {
        wristRight.setPosition(.8206);
    }
    
    public void upWristBottom() {
        wristRight.setPosition(.0978);
    }
    
    public void openClawBottom() {
        clawServoBottom.setPosition(.3383);
    }
    
    public void closeClawBottom() {
        clawServoBottom.setPosition(.5139);
    }
    
    public void tightCloseClawBottom() {
        clawServoBottom.setPosition(.5978);
    }
    
    public void startPivotBottom () {
        bottomPivot.setPosition(.0367);
    }
    
    public void endPivotBottom () {
        bottomPivot.setPosition(.4033);
    }
}
