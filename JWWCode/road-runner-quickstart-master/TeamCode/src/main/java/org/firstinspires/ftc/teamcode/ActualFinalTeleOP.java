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
            changeServoPositionBy(bottomPivot, .001);
        } else if (gamepad1.right_trigger > 0) {
            changeServoPositionBy(bottomPivot, -.001 );
        }
        // bottom claw
        if (gamepad1.left_bumper) {
            changeServoPositionBy(clawServoBottom, .001);
        } else if (gamepad1.right_bumper) {
            changeServoPositionBy(clawServoBottom, -.001);
        }

        // GAMEPAD 2

        // wrist
        if (gamepad2.right_trigger > 0) {
            changeServoPositionBy(wristLeft, .001);
            changeServoPositionBy(wristRight, -.001);
        } else if (gamepad2.left_trigger > 0) {
            changeServoPositionBy(wristLeft, -.001);
            changeServoPositionBy(wristRight, .001);
        }
        // top claw
        if (gamepad2.right_bumper) {
            changeServoPositionBy(clawServoTop, .001);
        } else if(gamepad2.left_bumper) {
            changeServoPositionBy(clawServoTop, -.001);
        }
        // claw wrist
        if (gamepad2.dpad_up) {
            changeServoPositionBy(clawWrist, .001);
        } else if (gamepad2.dpad_down) {
            changeServoPositionBy(clawWrist, -.001);
        }
        // extend
        if (gamepad2.dpad_right) {
            changeServoPositionBy(extendServo, -.001);
        } else if (gamepad2.dpad_left) {
            changeServoPositionBy(extendServo, .001);
        }
        // shoulder
        if (gamepad2.y) {
            changeServoPositionBy(shoulderLeft, .001);
            changeServoPositionBy(shoulderRight, -.001);
        } else if (gamepad2.a) {
            changeServoPositionBy(shoulderLeft, -.001);
            changeServoPositionBy(shoulderRight, .001);
        }
        // top pivot
        if (gamepad2.left_trigger > 0) {
            changeServoPositionBy(topPivot, .001);
        } else if (gamepad2.right_trigger > 0) {
            changeServoPositionBy(topPivot, -.001 );
        }


//        if (gamepad2.dpad_left) { // retract extend
//            try {
//                Thread.sleep(200);
//            } catch (InterruptedException e) {
//                // Handle the interrupted exception
//                Thread.currentThread().interrupt(); // Re-interrupt the thread
//            }
//        }

        updateAllTelemetry();
    }

    private void changeServoPositionBy(Servo servo, double delta) {
        servo.setPosition(servo.getPosition() + delta);
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

        // Calculate motor powers
//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//        double frontLeftPower = (y + x + rx) / denominator;
//        double backLeftPower = (y - x + rx) / denominator;
//        double frontRightPower = (y - x - rx) / denominator;
//        double backRightPower = (y + x - rx) / denominator;
        // different drive train
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
        // SLIDES movement code
//         if (gamepad2.left_stick_y < 0 && leftSlideMotor.getCurrentPosition() < 8100 && rightSlideMotor.getCurrentPosition() > -8100) {
//             rightSlideMotor.setPower(-gamepad2.left_stick_y * -.8);
//             leftSlideMotor.setPower(-gamepad2.left_stick_y * .8);
//         } else if (gamepad2.left_stick_y > 0 && leftSlideMotor.getCurrentPosition() > 0 && rightSlideMotor.getCurrentPosition() < 0) {
//             rightSlideMotor.setPower(-gamepad2.left_stick_y * -.8);
//             leftSlideMotor.setPower(-gamepad2.left_stick_y * .8);
//         } else {
//             rightSlideMotor.setPower(0);
//             leftSlideMotor.setPower(0);
//         }
        // Debugging COMMENT LATER ON
        rightSlideMotor.setPower(-gamepad2.left_stick_y * -1);
        leftSlideMotor.setPower(-gamepad2.left_stick_y * 1);
    }

    public void openClawTop() {

    }

    public void closeClawTop() {

    }

    public void startPivotTop () {

    }

    public void endPivotTop() {

    }

    public void startShould() {

    }

    public void basketShoulder() {

    }

    public void specimenGrabShoulder() {

    }

    public void specimenScoreShoulder() {

    }

    public void extend() {

    }

    public void retract() {

    }

    public void downWristBottom() {

    }

    public void upWristBottom() {

    }

    public void openClawBottom() {

    }

    public void closeClawBottom() {

    }

    public void startPivotBottom () {

    }

}