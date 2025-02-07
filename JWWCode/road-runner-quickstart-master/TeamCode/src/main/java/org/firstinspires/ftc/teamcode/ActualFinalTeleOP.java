package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "ActualFinalTeleOP", group = "TeleOp")
public class ActualFinalTeleOP extends OpMode {
    // Define motors for driving
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // motors for slides
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

    boolean isWristScanning = false;


    @Override
    public void init() {
        // Drivetrain motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Reverse left motors for proper directionality
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // left and right slide motors initialized
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

    @Override
    public void start() {

    }


    @Override
    public void loop() {
        controlDrivetrain();
        controlSlides();

        // GAMEPAD 1
        // top pivot
        if (gamepad1.left_trigger > 0) {
            endPivotBottom();
        } else if (gamepad1.right_trigger > 0) {
            startPivotBottom();
        }
        // bottom claw
        if (gamepad1.left_bumper) {
            closeClawBottom();
        } else if (gamepad1.right_bumper) {
            openClawBottom();
        }

        if (gamepad1.y) {
            basketShoulder();
            sampleScoreWristTop();
        }

        // GAMEPAD 2

        // wrist
        if (gamepad2.right_trigger > 0 && !isWristScanning) {
            isWristScanning = true;
            scanWristBottom();
        } else if (gamepad2.right_trigger > 0 && isWristScanning) {
            isWristScanning = false;
            downWristBottom();
        } else if (gamepad2.left_trigger > 0) {
            isWristScanning = false;
            upWristBottom();
        }
        // top claw
        if (gamepad2.right_bumper) {
            openClawTop();
        } else if(gamepad2.left_bumper) {
            closeClawTop();
        }

        // shoulder
        if (gamepad2.y) {
            try {
                openClawTop();
                Thread.sleep(100);
                startShoulder();
                sampleGrabWristTop();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        } else if (gamepad2.a) {
            try {
                openClawTop();
                sampleGrabShoulder();
                sampleGrabWristTop();
                Thread.sleep(100);
                closeClawTop();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }

        }

        if (gamepad2.b) {
            specimenScoreShoulder();
            specimenScoreWristTop();
            endPivotTop();
        } else if (gamepad2.x) {
            specimenGrabWristTop();
            specimenGrabShoulder();
            startPivotTop();
        }

        if (gamepad2.dpad_left) { // retract extend
            try {
                middleWristBottom();
                Thread.sleep(200);
                tightCloseClawBottom();
                Thread.sleep(100);
                upWristBottom();
                retract();
            } catch (InterruptedException e) {
                // Handle the interrupted exception
                Thread.currentThread().interrupt(); // Re-interrupt the thread
            }
        } else if (gamepad2.dpad_right) {
            extend();
        }

        updateAllTelemetry();
    }

    private void updateAllTelemetry() {
        telemetry.addData("extend: ", extendServo.getPosition());
        telemetry.addData("claw wrist: ", clawWrist.getPosition());
        telemetry.addData("Left slide: ", leftSlideMotor.getCurrentPosition());
        telemetry.addData("Right slide: ", rightSlideMotor.getCurrentPosition());
        telemetry.addData("Left wrist: ", wristLeft.getPosition());
        telemetry.addData("Right wrist: ", wristRight.getPosition());
        telemetry.addData("Left Shoulder arm: ", shoulderLeft.getPosition());
        telemetry.addData("Right Shoulder arm: ", shoulderRight.getPosition());
        telemetry.addData("Claw servo bottom: ", clawServoBottom.getPosition());
        telemetry.addData("Claw servo top: ", clawServoTop.getPosition());
        telemetry.addData("Top pivot bottom: ", topPivot.getPosition());
        telemetry.addData("Bottom pivot top: ", bottomPivot.getPosition());
        telemetry.update();
    }

    private void controlDrivetrain() {
        double y = -gamepad1.left_stick_y;  // Forward/backward
        double x = gamepad1.left_stick_x * 1.1;  // Strafe
        double rx = gamepad1.right_stick_x * .6;  // Rotation

        double frontLeftPower = (y + x + rx) * .9;
        double frontRightPower = (y - x - rx) * .9;
        double backLeftPower = (y - x + rx) * .9;
        double backRightPower = (y + x - rx) * .9;

        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
    }

    private void controlSlides() {
        rightSlideMotor.setPower(-gamepad2.left_stick_y * -1);
        leftSlideMotor.setPower(-gamepad2.left_stick_y * 1);
    }

    public void openClawTop() {
        clawServoTop.setPosition(.0);
    }

    public void closeClawTop() {
        clawServoTop.setPosition(.0);
    }

    public void startPivotTop () {
        topPivot.setPosition(.7378);
    }

    public void endPivotTop() {
        topPivot.setPosition(.0361);
    }

    public void sampleGrabWristTop() {
        clawWrist.setPosition(.0);
    }

    public void specimenGrabWristTop() {
        clawWrist.setPosition(.0);
    }

    public void sampleScoreWristTop() {
        clawWrist.setPosition(.0);
    }

    public void specimenScoreWristTop () {
        clawWrist.setPosition(.0);
    }

    public void startShoulder() {
        shoulderLeft.setPosition(.0);
        shoulderRight.setPosition(.0);
    }

    public void sampleGrabShoulder() {
        shoulderLeft.setPosition(.0);
        shoulderRight.setPosition(.0);
    }

    public void basketShoulder() {
        shoulderLeft.setPosition(.0);
        shoulderRight.setPosition(.0);
    }

    public void specimenGrabShoulder() {
        shoulderLeft.setPosition(.0);
        shoulderRight.setPosition(.0);
    }

    public void specimenScoreShoulder() {
        shoulderLeft.setPosition(.0);
        shoulderRight.setPosition(.0);
    }

    public void extend() {
        extendServo.setPosition(.0);
    }

    public void retract() {
        extendServo.setPosition(.0);
    }

    public void downWristBottom() {
        wristRight.setPosition(.9117);
    }

    public void middleWristBottom() {
        wristRight.setPosition(.0);
    }

    public void scanWristBottom() {
        wristRight.setPosition(.8739);
    }

    public void upWristBottom() {
        wristRight.setPosition(.0);
    }

    public void openClawBottom() {
        clawServoBottom.setPosition(.2978);
    }

    public void closeClawBottom() {
        clawServoBottom.setPosition(.7128);
    }

    public void tightCloseClawBottom() {
        clawServoBottom.setPosition(.0);
    }

    public void startPivotBottom () {
        bottomPivot.setPosition(.0367);
    }

    public void endPivotBottom () {
        bottomPivot.setPosition(.4033);
    }

}