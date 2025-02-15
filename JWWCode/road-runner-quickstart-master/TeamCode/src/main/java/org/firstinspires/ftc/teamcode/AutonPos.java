package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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


//    public Action openClawTop() {
//        clawServoTop.setPosition(.3511);
//        return null;
//    }

    public Action openClawTop() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return false;
            }

            public boolean run(double deltaTime) {
                clawServoTop.setPosition(0.3511);
                return true; // This ensures the action completes instantly
            }

            public void start() {}

            public void stop() {}

            public boolean isFinished() {
                return true; // Instantly finishes
            }
        };
    }


    public Action closeClawTop() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return false;
            }

            public boolean run(double deltaTime) {
                clawServoTop.setPosition(0.0889);
                return true; // This ensures the action completes instantly
            }

            public void start() {}

            public void stop() {}

            public boolean isFinished() {
                return true; // Instantly finishes
            }
        };
    }

    public Action startPivotTop () {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return false;
            }

            public boolean run(double deltaTime) {
                clawServoTop.setPosition(0.7328);
                return true; // This ensures the action completes instantly
            }

            public void start() {}

            public void stop() {}

            public boolean isFinished() {
                return true; // Instantly finishes
            }
        };
    }

    public Action endPivotTop() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return false;
            }

            public boolean run(double deltaTime) {
                clawServoTop.setPosition(0.0322);
                return true; // This ensures the action completes instantly
            }

            public void start() {}

            public void stop() {}

            public boolean isFinished() {
                return true; // Instantly finishes
            }
        };
    }

    public Action sampleGrabWristTop() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return false;
            }

            public boolean run(double deltaTime) {
                clawServoTop.setPosition(0.0744);
                return true; // This ensures the action completes instantly
            }

            public void start() {}

            public void stop() {}

            public boolean isFinished() {
                return true; // Instantly finishes
            }
        };
    }

    public Action specimenGrabWristTop() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return false;
            }

            public boolean run(double deltaTime) {
                clawServoTop.setPosition(0.4883);
                return true; // This ensures the action completes instantly
            }

            public void start() {}

            public void stop() {}

            public boolean isFinished() {
                return true; // Instantly finishes
            }
        };
    }

    public Action sampleScoreWristTop() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return false;
            }

            public boolean run(double deltaTime) {
                clawServoTop.setPosition(0.6856);
                return true; // This ensures the action completes instantly
            }

            public void start() {}

            public void stop() {}

            public boolean isFinished() {
                return true; // Instantly finishes
            }
        };
    }

    public Action specimenScoreWristTop () {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return false;
            }

            public boolean run(double deltaTime) {
                clawServoTop.setPosition(0.15);
                return true; // This ensures the action completes instantly
            }

            public void start() {}

            public void stop() {}

            public boolean isFinished() {
                return true; // Instantly finishes
            }
        };
    }

    public Action startShoulder() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return false;
            }

            public boolean run(double deltaTime) {
                shoulderLeft.setPosition(0.6861);
                shoulderRight.setPosition(0.2661);
                return true; // This ensures the action completes instantly
            }

            public void start() {}

            public void stop() {}

            public boolean isFinished() {
                return true; // Instantly finishes
            }
        };
    }

    public Action sampleGrabShoulder() {
        shoulderLeft.setPosition(.8828);
        shoulderRight.setPosition(.0717);
        return null;
    }

    public Action basketShoulder() {
        shoulderLeft.setPosition(.4111);
        shoulderRight.setPosition(.5839);
        return null;
    }

    public Action specimenGrabShoulder() {
        shoulderLeft.setPosition(.8983);
        shoulderRight.setPosition(.05);
        return null;
    }

    public Action specimenScoreShoulder() {
        shoulderLeft.setPosition(.1461);
        shoulderRight.setPosition(.8011);
        return null;
    }

    public Action retract() {
        extendServo.setPosition(0);
        return null;
    }

    public Action extendForm() {
        extendServo.setPosition(.56);
        return null;
    }

    public Action downWristBottom() {
        wristRight.setPosition(.9222);
        return null;
    }

    public Action middleWristBottom() {
        wristRight.setPosition(.5116);
        return null;
    }

    public Action scanWristBottom() {
        wristRight.setPosition(.8206);
        return null;
    }

    public Action upWristBottom() {
        wristRight.setPosition(.0978);
        return null;
    }

    public Action openClawBottom() {
        clawServoBottom.setPosition(.3383);
        return null;
    }

    public Action closeClawBottom() {
        clawServoBottom.setPosition(.5139);
        return null;
    }

    public Action tightCloseClawBottom() {
        clawServoBottom.setPosition(.5978);
        return null;
    }

    public Action startPivotBottom () {
        bottomPivot.setPosition(.0367);
        return null;
    }

    public Action endPivotBottom () {
        bottomPivot.setPosition(.4033);
        return null;
    }
    public void loop() {
        // Needed to add this statement
    }
}


