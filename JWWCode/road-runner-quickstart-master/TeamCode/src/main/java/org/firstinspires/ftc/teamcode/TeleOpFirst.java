//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//
//@TeleOp(name = "PIDTeleOpFinal", group = "TeleOp")
//public class PIDTeleOp extends OpMode {
//
//    // PID constants
//    // Proportional gain
//    double kp = 0.01;
//    // Integral gain
//    double ki = 0.0;
//    // Derivative gain
//    double kd = 0.001;
//
//    private long delayStartTime = 0;
//    private boolean isDelaying = false;
//
//    // PID variables
//    private double targetPosition = 0; // Desired encoder position
//    private double integralSum = 0;
//    private double lastError = 0;
//    private double currentPosition = 0;
//    private double power = 0;
//
//    // Constants for slides
//    private final int BASKET_SLIDE_POSITION = 3000;     // Maximum encoder ticks (full extension)
//    private final int SPECIMEN_HANG_POS = 700;
//    private final int SPECIMEN_GRAB_POS = 25;
//    private final int MIN_POSITION = 0;       // Minimum encoder ticks (fully retracted)
//    private final int NORMAL_INCREMENT = 50;   // Encoder ticks per loop for continuous motion
//
//    // Define motors for driving
//    private DcMotor frontLeft;
//    private DcMotor frontRight;
//    private DcMotor backLeft;
//    private DcMotor backRight;
//
//    // motors for slides
//    private DcMotor leftSlideMotor;
//    private DcMotor rightSlideMotor;
//
//    // Servos
//    private Servo topPivot;
//    private Servo bottomPivot;
//    private Servo shoulderLeft;
//    private Servo shoulderRight;
//    private Servo clawServoBottom;
//    private Servo clawServoTop;
//    private Servo clawWrist;
//
//    private Servo extendServo;
//    private Servo wristRight;
//
//    private boolean isDrivetrainReversed = false;
//
//    private double prevFrontLeftPower = 0;
//    private double prevFrontRightPower = 0;
//    private double prevBackLeftPower = 0;
//    private double prevBackRightPower = 0;
//
//    // Slew rate factor: Adjust this for more/less smoothness (lower = smoother)
//    private final double SLEW_RATE = 0.05;
//
//    @Override
//    public void init() {
//        // Drivetrain motors
//        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
//        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
//        backRight = hardwareMap.get(DcMotor.class, "backRight");
//
//        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        // Reverse left motors for proper directionality
//        frontLeft.setDirection(DcMotor.Direction.REVERSE);
//        backLeft.setDirection(DcMotor.Direction.REVERSE);
//
//        // left and right slide motors initialized
//        leftSlideMotor = hardwareMap.get(DcMotor.class, "leftSlideMotor");
//        rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");
//
//        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        leftSlideMotor.setDirection(DcMotor.Direction.REVERSE);
//
//        leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        shoulderLeft = hardwareMap.get(Servo.class, "shoulderLeft");
//        shoulderRight = hardwareMap.get(Servo.class, "shoulderRight");
//
//        extendServo = hardwareMap.get(Servo.class, "extendServo");
//
//        wristRight = hardwareMap.get(Servo.class, "wristRight");
//
//        clawServoBottom = hardwareMap.get(Servo.class, "clawServoBottom");
//        clawServoTop = hardwareMap.get(Servo.class, "clawServoTop");
//
//        topPivot = hardwareMap.get(Servo.class, "topPivot");
//        bottomPivot = hardwareMap.get(Servo.class, "bottomPivot");
//
//        clawWrist = hardwareMap.get(Servo.class, "clawWrist");
//    }
//
//    @Override
//    public void start() {
//        closeClawTop();
//        startShoulder();
//        sampleGrabWristTop();
//        upWristBottom();
//        openClawBottom();
//        startPivotBottom();
//        startPivotTop();
//        retract();
//    }
//
//
//    @Override
//    public void loop() {
//        if (gamepad2.dpad_up) targetPosition += NORMAL_INCREMENT; // Increase target position
//        else if (gamepad2.dpad_down) targetPosition -= NORMAL_INCREMENT; // Decrease target position
//
//        runSlidePID(); // Slide PID
//        controlDrivetrain(); // Drivetrain
//
//        // Top Pivot Control
//        if (gamepad1.right_trigger > 0) startPivotTop();
//        else if (gamepad1.left_trigger > 0) endPivotTop();
//
//        if (gamepad1.b) isDrivetrainReversed = !isDrivetrainReversed;
//
//        // Bottom pivot Control
//        if (gamepad2.x) startPivotBottom();
//        else if (gamepad2.b) endPivotBottom();
//
//        // Bottom claw Control
//        if (gamepad2.left_bumper) {
//            downWristBottom();
//            startDelay(500);
//            if (delayCompleted()) closeClawBottom();
//        } else if (gamepad2.right_bumper) openClawBottom();
//
//        // Wrist Controls
//        if (gamepad2.right_trigger > 0) {
//            scanWristBottom();
//            openClawBottom();
//        } else if (gamepad2.left_trigger > 0) upWristBottom();
//
//        // Top claw Controls
//        if (gamepad1.right_bumper) openClawTop();
//        else if(gamepad1.left_bumper) closeClawTop();
//
//        // Rest Position
//        if (gamepad1.a) {
//            targetPosition = MIN_POSITION;
//            openClawTop();
//            startDelay(100);
//            if (delayCompleted()) {
//                startShoulder();
//                sampleGrabWristTop();
//            }
//        }
//
//        if (gamepad2.y) { // Basket Position
//            targetPosition = BASKET_SLIDE_POSITION;
//            basketShoulder();
//            sampleScoreWristTop();
//        } else if (gamepad2.a) { // Grab Sample
//            openClawTop();
//            startDelay(500);
//            if (delayCompleted()) {
//                sampleGrabShoulder();
//                sampleGrabWristTop();
//                startDelay(500);
//                if (delayCompleted()) {
//                    closeClawTop();
//                    startDelay(500);
//                    if (delayCompleted()) openClawBottom();
//                }
//            }
//        }
//
//        if (gamepad1.y) { // Specimen Hang Position
//            specimenScoreShoulder();
//            specimenScoreWristTop();
//            endPivotTop();
//            targetPosition = SPECIMEN_HANG_POS;
//        } else if (gamepad1.x) { // Specimen Grab Position
//            specimenGrabWristTop();
//            specimenGrabShoulder();
//            startPivotTop();
//            targetPosition = SPECIMEN_GRAB_POS;
//        }
//
//        // Retract
//        if (gamepad2.dpad_left) {
//            startPivotBottom();
//            middleWristBottom();
//            startDelay(700);
//            if (delayCompleted()) {
//                tightCloseClawBottom();
//                startDelay(100);
//                if (delayCompleted()) {
//                    upWristBottom();
//                    retract();
//                }
//            }
//        } else if (gamepad2.dpad_right) { // Extend
//            extend();
//        }
//
//        updateTelemetry();
//    }
//
//    private void startDelay(long milliseconds) {
//        delayStartTime = System.currentTimeMillis() + milliseconds;
//        isDelaying = true;
//    }
//
//    private boolean delayCompleted() {
//        if (isDelaying && System.currentTimeMillis() >= delayStartTime) {
//            isDelaying = false;
//            return true;
//        }
//        return false;
//    }
//
//    public void updateTelemetry() {
//        telemetry.addLine("--------------------------------------------- Servos ---------------------------------------------");
//        telemetry.addData("Claw servo bottom: ", clawServoBottom.getPosition());
//        telemetry.addData("Bottom pivot top: ", bottomPivot.getPosition());
//        telemetry.addData("Bottom wrist: ", wristRight.getPosition());
//        telemetry.addData("Extend: ", extendServo.getPosition());
//        telemetry.addData("Left Shoulder arm: ", shoulderLeft.getPosition());
//        telemetry.addData("Right Shoulder arm: ", shoulderRight.getPosition());
//        telemetry.addData("Claw wrist: ", clawWrist.getPosition());
//        telemetry.addData("Top pivot bottom: ", topPivot.getPosition());
//        telemetry.addData("Claw servo top: ", clawServoTop.getPosition());
//        telemetry.addLine("--------------------------------------------- Slide PID ---------------------------------------------");
//        telemetry.addData("Target Position", targetPosition);
//        telemetry.addData("Current Position", currentPosition);
//        telemetry.addData("Slide Motor Power", power);
//        telemetry.addLine("--------------------------------------------- Drivetrain ---------------------------------------------");
//        telemetry.addData("Drivetrain Reversed", isDrivetrainReversed);
//        telemetry.update();
//    }
//
//    private void SetDrivetrainMotorPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
//        frontLeft.setPower(frontLeftPower);
//        frontRight.setPower(frontRightPower);
//        backLeft.setPower(backLeftPower);
//        backRight.setPower(backRightPower);
//    }
//
//    private void controlDrivetrain() {
//        double frontLeftFactor = 1;
//        double frontRightFactor = 1;
//        double backLeftFactor = 1;
//        double backRightFactor = 1;
//
//        double y = -gamepad1.left_stick_y;  // Forward/backward
//        double x = gamepad1.left_stick_x * 1.1;  // Strafe
//        double rx = gamepad1.right_stick_x * .6;  // Rotation
//        if (gamepad1.right_stick_button)  rx = gamepad1.right_stick_x * .3;
//        double deadzone = 0.1; // tighter stick control
//        y = Math.abs(y) > deadzone ? y : 0;
//        x = Math.abs(x) > deadzone ? x : 0;
//        rx = Math.abs(rx) > deadzone ? rx : 0;
//
//        double factor = isDrivetrainReversed ? -1 : 1;
//        double targetFrontLeftPower = (y + x + rx) * factor * frontLeftFactor;
//        double targetFrontRightPower = (y - x - rx) * factor * frontRightFactor;
//        double targetBackLeftPower = (y - x + rx) * factor * backLeftFactor;
//        double targetBackRightPower = (y + x - rx) * factor * backRightFactor;
//
//        // Find the maximum power applied
//        double maxPower = Math.max(Math.abs(targetFrontLeftPower), Math.max(Math.abs(targetFrontRightPower),
//                Math.max(Math.abs(targetBackLeftPower), Math.abs(targetBackRightPower))));
//
//        // Normalize if any power exceeds 1
//        if (maxPower > 1.0) {
//            targetFrontLeftPower /= maxPower;
//            targetFrontRightPower /= maxPower;
//            targetBackLeftPower /= maxPower;
//            targetBackRightPower /= maxPower;
//        }
//
//        // Apply slew rate limiting for smooth transitions
//        prevFrontLeftPower += clip(targetFrontLeftPower - prevFrontLeftPower, -SLEW_RATE, SLEW_RATE);
//        prevFrontRightPower += clip(targetFrontRightPower - prevFrontRightPower, -SLEW_RATE, SLEW_RATE);
//        prevBackLeftPower += clip(targetBackLeftPower - prevBackLeftPower, -SLEW_RATE, SLEW_RATE);
//        prevBackRightPower += clip(targetBackRightPower - prevBackRightPower, -SLEW_RATE, SLEW_RATE);
//
//        SetDrivetrainMotorPowers(prevFrontLeftPower, prevFrontRightPower, prevBackLeftPower, prevBackRightPower);
//    }
//
//    private double clip(double value, double min, double max) {
//        return Math.max(min, Math.min(max, value));
//    }
//
//    public void runSlidePID() {
//        // Clamp target position within bounds
//        targetPosition = Math.max(MIN_POSITION, Math.min(BASKET_SLIDE_POSITION, targetPosition));
//
//        // Get the current position from encoders
//        double currentPosition = (leftSlideMotor.getCurrentPosition() + rightSlideMotor.getCurrentPosition()) / 2.0;
//
//        // Calculate PID terms
//        double error = targetPosition - currentPosition;
//        integralSum += error; // Accumulate error for integral term
//        double derivative = error - lastError; // Change in error for derivative term
//
//        // Compute motor power using PID
//        double power = (kp * error) + (ki * integralSum) + (kd * derivative);
//
//        // Apply power to both motors
//        leftSlideMotor.setPower(power);
//        rightSlideMotor.setPower(power);
//
//        // Update last error
//        lastError = error;
//    }
//
//    // Positions:
//
//    public void openClawTop() {
//        clawServoTop.setPosition(.3511);
//    }
//
//    public void closeClawTop() {
//        clawServoTop.setPosition(0.0889);
//    }
//
//    public void startPivotTop () {
//        topPivot.setPosition(.7328);
//    }
//
//    public void endPivotTop() {
//        topPivot.setPosition(.0322);
//    }
//
//    public void sampleGrabWristTop() {
//        clawWrist.setPosition(.0744);
//    }
//
//    public void specimenGrabWristTop() {
//        clawWrist.setPosition(.4883);
//    }
//
//    public void sampleScoreWristTop() {
//        clawWrist.setPosition(.6856);
//    }
//
//    public void specimenScoreWristTop () {
//        clawWrist.setPosition(.15);
//    }
//
//    public void startShoulder() {
//        shoulderLeft.setPosition(.1528);
//        shoulderRight.setPosition(.73);
//    }
//
//    public void sampleGrabShoulder() {
//        shoulderLeft.setPosition(0);
//        shoulderRight.setPosition(.9156);
//    }
//
//    public void basketShoulder() {
//        shoulderLeft.setPosition(.4817);
//        shoulderRight.setPosition(.4083);
//    }
//
//    public void specimenGrabShoulder() {
//        shoulderLeft.setPosition(.03);
//        shoulderRight.setPosition(.97);
//    }
//
//    public void specimenScoreShoulder() {
//        shoulderLeft.setPosition(.7372);
//        shoulderRight.setPosition(.1383);
//    }
//
//    public void retract() {
//        extendServo.setPosition(0);
//    }
//
//    public void extend() {
//        extendServo.setPosition(.56);
//    }
//
//    public void downWristBottom() {
//        wristRight.setPosition(.9222);
//    }
//
//    public void middleWristBottom() {
//        wristRight.setPosition(.5116);
//    }
//
//    public void scanWristBottom() {
//        wristRight.setPosition(.8206);
//    }
//
//    public void upWristBottom() {
//        wristRight.setPosition(.0978);
//    }
//
//    public void openClawBottom() {
//        clawServoBottom.setPosition(.3383);
//    }
//
//    public void closeClawBottom() {
//        clawServoBottom.setPosition(.5139);
//    }
//
//    public void tightCloseClawBottom() {
//        clawServoBottom.setPosition(.5978);
//    }
//
//    public void startPivotBottom () {
//        bottomPivot.setPosition(.0367);
//    }
//
//    public void endPivotBottom () {
//        bottomPivot.setPosition(.4033);
//    }
//
//}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "PIDTeleOpFinal", group = "TeleOp")
public class TeleOpFirst extends OpMode {

    // PID constants
    // Proportional gain
    double kp = 0.01;
    // Integral gain
    double ki = 0.0;
    // Derivative gain
    double kd = 0.001;

//    private long delayStartTime = 0;
//    private boolean isDelaying = false;

    // PID variables
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
    private Servo wristRight;

    private boolean isDrivetrainReversed = false;

    private double prevFrontLeftPower = 0;
    private double prevFrontRightPower = 0;
    private double prevBackLeftPower = 0;
    private double prevBackRightPower = 0;

    // Slew rate factor: Adjust this for more/less smoothness (lower = smoother)
    private final double SLEW_RATE = 0.05;

    @Override
    public void init() {
        // Drivetrain motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reverse left motors for proper directionality
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // left and right slide motors initialized
        leftSlideMotor = hardwareMap.get(DcMotor.class, "leftSlideMotor");
        rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");

        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftSlideMotor.setDirection(DcMotor.Direction.REVERSE);

        leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    @Override
    public void start() {
        closeClawTop();
        startShoulder();
        sampleGrabWristTop();
        upWristBottom();
        openClawBottom();
        startPivotBottom();
        startPivotTop();
        retract();
    }

    private void changeServoPositionBy(Servo servo, double delta) {
        servo.setPosition(servo.getPosition() + delta);
    }
    @Override
    public void loop() {
        if (gamepad2.dpad_up) targetPosition += 25; // Increase target position
        else if (gamepad2.dpad_down) targetPosition -= 25; // Decrease target position

//        if (gamepad1.dpad_up) changeServoPositionBy(clawWrist, .002);
//        else if (gamepad1.dpad_down) changeServoPositionBy(clawWrist, -.002);

        runSlidePID(); // Slide PID
        controlDrivetrain(); // Drivetrain

        // Top Pivot Control
        // Top Pivot Control
        if (gamepad1.right_trigger > 0) startPivotTop();
        else if (gamepad1.left_trigger > 0) endPivotTop();

        if (gamepad1.b) isDrivetrainReversed = !isDrivetrainReversed;

        // Bottom pivot Control
        if (gamepad2.x) startPivotBottom();
        else if (gamepad2.b) endPivotBottom();

        // Bottom claw Control
        if (gamepad2.left_bumper) {
            downWristBottom();
            delay(500);
            closeClawBottom();
        } else if (gamepad2.right_bumper) openClawBottom();

        // Wrist Controls
        if (gamepad2.right_trigger > 0) {
            scanWristBottom();
            openClawBottom();
        } else if (gamepad2.left_trigger > 0) upWristBottom();

        // Top claw Controls
        if (gamepad1.right_bumper) openClawTop();
        else if(gamepad1.left_bumper) closeClawTop();

        // Rest Position
        if (gamepad1.a) {
            targetPosition = MIN_POSITION;
            openClawTop();
            delay(100);
            startShoulder();
            sampleGrabWristTop();
        }

        if (gamepad2.y) { // Basket Position
            targetPosition = BASKET_SLIDE_POSITION;
            basketShoulder();
            sampleScoreWristTop();
        } else if (gamepad2.a) { // Grab Sample
            openClawTop();
            delay(500);
            sampleGrabShoulder();
            sampleGrabWristTop();
            delay(500);
            closeClawTop();
            delay(500);
            openClawBottom();
        }

        if (gamepad1.y) { // Specimen Hang Position
            specimenScoreShoulder();
            specimenScoreWristTop();
            endPivotTop();
            targetPosition = SPECIMEN_HANG_POS;
        } else if (gamepad1.x) { // Specimen Grab Position
            specimenGrabWristTop();
            specimenGrabShoulder();
            startPivotTop();
            targetPosition = SPECIMEN_GRAB_POS;
        }

        // Retract
        if (gamepad2.dpad_left) {
            startPivotBottom();
            middleWristBottom();
            delay(700);
            tightCloseClawBottom();
            delay(100);
            upWristBottom();
            retract();
        } else if (gamepad2.dpad_right) { // Extend
            extend();
        }

        updateTelemetry();
    }

    private void delay(long milliseconds) {
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < milliseconds) continue;
    }

    public void updateTelemetry() {
        telemetry.addLine("--------------------------------------------- Servos ---------------------------------------------");
        telemetry.addData("Claw servo bottom: ", clawServoBottom.getPosition());
        telemetry.addData("Bottom pivot top: ", bottomPivot.getPosition());
        telemetry.addData("Bottom wrist: ", wristRight.getPosition());
        telemetry.addData("Extend: ", extendServo.getPosition());
        telemetry.addData("Left Shoulder arm: ", shoulderLeft.getPosition());
        telemetry.addData("Right Shoulder arm: ", shoulderRight.getPosition());
        telemetry.addData("Claw wrist: ", clawWrist.getPosition());
        telemetry.addData("Top pivot bottom: ", topPivot.getPosition());
        telemetry.addData("Claw servo top: ", clawServoTop.getPosition());
        telemetry.addLine("--------------------------------------------- Slide PID ---------------------------------------------");
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("Slide Motor Power", power);
        telemetry.addLine("--------------------------------------------- Drivetrain ---------------------------------------------");
        telemetry.addData("Drivetrain Reversed", isDrivetrainReversed);
        telemetry.update();
    }

    private void SetDrivetrainMotorPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    private void controlDrivetrain() {
//        double frontLeftFactor = 1;
//        double frontRightFactor = 1;
//        double backLeftFactor = 1;
//        double backRightFactor = 1;
//        if (gamepad1.left_stick_button) {
        double frontLeftFactor = .2;
        double frontRightFactor = .2;
        double backLeftFactor = .2;
        double backRightFactor = .2;
//        }

        if (gamepad1.dpad_up) SetDrivetrainMotorPowers(frontLeftFactor, frontRightFactor, backLeftFactor, backRightFactor);
        else if (gamepad1.dpad_down) SetDrivetrainMotorPowers(-frontLeftFactor, -frontRightFactor, -backLeftFactor, -backRightFactor);
        else if (gamepad1.dpad_left) SetDrivetrainMotorPowers(-frontLeftFactor, frontRightFactor, backLeftFactor, -backRightFactor);
        else if (gamepad1.dpad_right) SetDrivetrainMotorPowers(frontLeftFactor,-frontRightFactor, -backLeftFactor, backRightFactor);

        double y = -gamepad1.left_stick_y;  // Forward/backward
        double x = gamepad1.left_stick_x * 1.1;  // Strafe
        double rx = gamepad1.right_stick_x * .6;  // Rotation
        if (gamepad1.right_stick_button)  rx = gamepad1.right_stick_x * .2;
        double deadzone = 0.1; // tighter stick control
        y = Math.abs(y) > deadzone ? y : 0;
        x = Math.abs(x) > deadzone ? x : 0;
        rx = Math.abs(rx) > deadzone ? rx : 0;
        double factor = 1;
        if (gamepad1.left_stick_button) factor = .1;
//        double factor = isDrivetrainReversed ? -1 : 1;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = ((y + x + rx) / denominator) * factor;
        double backLeftPower = ((y - x + rx) / denominator) * factor;
        double frontRightPower = ((y - x - rx) / denominator) * factor;
        double backRightPower = ((y + x - rx) / denominator) * factor;
//        double targetFrontLeftPower = (y + x + rx) * factor * frontLeftFactor;
//        double targetFrontRightPower = (y - x - rx) * factor * frontRightFactor;
//        double targetBackLeftPower = (y - x + rx) * factor * backLeftFactor;
//        double targetBackRightPower = (y + x - rx) * factor * backRightFactor;

        // Find the maximum power applied
//        double maxPower = Math.max(Math.abs(targetFrontLeftPower), Math.max(Math.abs(targetFrontRightPower),
//                Math.max(Math.abs(targetBackLeftPower), Math.abs(targetBackRightPower))));
//p6
//        // Normalize if any power exceeds 1
//        if (maxPower > 1.0) {
//            targetFrontLeftPower /= maxPower;
//            targetFrontRightPower /= maxPower;
//            targetBackLeftPower /= maxPower;
//            targetBackRightPower /= maxPower;
//        }
//
//        // Apply slew rate limiting for smooth transitions
//        prevFrontLeftPower += clip(targetFrontLeftPower - prevFrontLeftPower, -SLEW_RATE, SLEW_RATE);
//        prevFrontRightPower += clip(targetFrontRightPower - prevFrontRightPower, -SLEW_RATE, SLEW_RATE);
//        prevBackLeftPower += clip(targetBackLeftPower - prevBackLeftPower, -SLEW_RATE, SLEW_RATE);
//        prevBackRightPower += clip(targetBackRightPower - prevBackRightPower, -SLEW_RATE, SLEW_RATE);

//        SetDrivetrainMotorPowers(prevFrontLeftPower, prevFrontRightPower, prevBackLeftPower, prevBackRightPower);
        SetDrivetrainMotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    private double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public void runSlidePID() {
        // Clamp target position within bounds
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
    }

    // Positions:

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
        clawWrist.setPosition(.1222);
    }

    public void specimenGrabWristTop() {
        clawWrist.setPosition(.5478);
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
        shoulderLeft.setPosition(.8467);
        shoulderRight.setPosition(.1017);
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
        clawServoBottom.setPosition(.3817);
    }

    public void closeClawBottom() {
        clawServoBottom.setPosition(.18);
    }

    public void tightCloseClawBottom() {
        clawServoBottom.setPosition(.1122);
    }

    public void startPivotBottom () {
        bottomPivot.setPosition(.0367);
    }

    public void endPivotBottom () {
        bottomPivot.setPosition(.4033);
    }

}