package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ASTeleOpFinal", group = "TeleOp")
public class TeleOpFirst extends OpMode {
    // Define motors for driving
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // motors for slides
    private DcMotor leftSlideMotor;
    private DcMotor rightSlideMotor;

    // Servos
    private Servo shoulderLeft;
    private Servo shoulderRight;
    private Servo elbowLeft;
    private Servo elbowRight;
    private Servo clawServo;
    private Servo extendLeft;
    private Servo extendRight;
    private Servo wristLeft;
    private Servo wristRight;
    private CRServo spinTakeLeft;
    private CRServo spinTakeRight;


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

        elbowLeft = hardwareMap.get(Servo.class, "elbowLeft");
        elbowRight = hardwareMap.get(Servo.class, "elbowRight");

        shoulderLeft = hardwareMap.get(Servo.class, "shoulderLeft");
        shoulderRight = hardwareMap.get(Servo.class, "shoulderRight");

        spinTakeLeft = hardwareMap.get(CRServo.class, "spinTakeLeft");
        spinTakeRight = hardwareMap.get(CRServo.class, "spinTakeRight");

        extendLeft = hardwareMap.get(Servo.class, "leftExtend");
        extendRight = hardwareMap.get(Servo.class, "rightExtend");

        wristRight = hardwareMap.get(Servo.class, "wristRight");
        wristLeft = hardwareMap.get(Servo.class, "wristLeft");

        clawServo = hardwareMap.get(Servo.class, "clawServo");
    }

    @Override
    public void start() {
        shoulderLeft.setPosition(0);
        shoulderRight.setPosition(1);
        elbowLeft.setPosition(9939);
        elbowRight.setPosition(.2667);
        retract();
        elbowLeft.setPosition(.9267);
        elbowRight.setPosition(.315);
        wristUp();
        clawServo.setPosition(.6794);
    }


    @Override
    public void loop() {
        controlDrivetrain();
        controlSlides();
        fineTuningControls();

//        if (leftSlideMotor.getCurrentPosition() > 15) {
//            retract();
//        }
        if (gamepad1.x) specimenHangSlides();

        if (gamepad1.dpad_up) retract();

        if (gamepad1.right_bumper) { // intake
            spinTakeRight.setPower(1);
            spinTakeLeft.setPower(-1);
        } else if (gamepad1.left_bumper) { // ex take
            spinTakeRight.setPower(-1);
            spinTakeLeft.setPower(1);
        } else {
            spinTakeRight.setPower(0);
            spinTakeLeft.setPower(0);
        }

        if (gamepad2.right_trigger > 0) wristUp(); // gets wrist up
        else if(gamepad2.left_trigger > 0) wristDown(); // turn wrist down

        if (gamepad1.right_trigger > 0) wristUp(); // gets wrist up
        else if(gamepad1.left_trigger > 0) wristDown(); // turn wrist down

        if (gamepad2.left_bumper) clawServo.setPosition(1); // close claw
        else if (gamepad2.right_bumper) clawServo.setPosition(.6794); // open claw

        if (gamepad1.b) armPositionSpecimenHang(); // extend arm for specimen hang

        if (gamepad2.x) armBasketPosition(); // extend arm for sample in basket
        else if (gamepad2.b) armStartPosition(); // retracted for specimen/sample grab

        if (gamepad2.dpad_left) { // retract extend
            try {
                clawServo.setPosition(.6794); // open claw
                wristUp();
                elbowUp();
                armStartPosition();
                retract();
                Thread.sleep(1000);
                elbowDown();
                spinTakeRight.setPower(1);
                spinTakeLeft.setPower(-1);
                Thread.sleep(1000);
                spinTakeRight.setPower(0);
                spinTakeLeft.setPower(0);
                clawServo.setPosition(1); // close claw
                Thread.sleep(500);
                extend();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt(); // Re-interrupt the thread
            }
        } else if (gamepad2.dpad_right) { // extend extend servos
            extend();
        }

//        if (gamepad1.x) retract();
        updateAllTelemetry();
    }

    private void changeServoPositionBy(Servo servo, double delta) {
        servo.setPosition(servo.getPosition() + delta);
    }

    private void updateAllTelemetry() {
        telemetry.addData("Left elbow servo arm: ", elbowLeft.getPosition());
        telemetry.addData("Right elbow servo arm: ", elbowRight.getPosition());
        telemetry.addData("Left extend: ", extendLeft.getPosition());
        telemetry.addData("Right extend: ", extendRight.getPosition());
        telemetry.addData("Left slide: ", leftSlideMotor.getCurrentPosition());
        telemetry.addData("Right slide: ", rightSlideMotor.getCurrentPosition());
        telemetry.addData("Left wrist: ", wristLeft.getPosition());
        telemetry.addData("Right wrist: ", wristRight.getPosition());
        telemetry.addData("Left Shoulder arm: ", shoulderLeft.getPosition());
        telemetry.addData("Right Shoulder arm: ", shoulderRight.getPosition());
        telemetry.addData("Claw servo: ", clawServo.getPosition());
        telemetry.update();
    }

    private void controlDrivetrain() {
//        if (gamepad1.dpad_up) SetDrivetrainMotorPowers(.9, .9, .9, .9);
        if (gamepad1.dpad_down) SetDrivetrainMotorPowers(-.9, -.9, -.9, -.9);
        else if (gamepad1.dpad_left) SetDrivetrainMotorPowers(-.9, .9, .9, -.9);
        else if (gamepad1.dpad_right) SetDrivetrainMotorPowers(.9, -.9, -.9, .9);
        else {
            // Default joystick control
            double y = -gamepad1.left_stick_y;  // Forward/backward
            double x = gamepad1.left_stick_x * 1.1;  // Strafe
            double rx = gamepad1.right_stick_x * .6;  // Rotation

//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//            double frontLeftPower = (y + x + rx) / denominator;
//            double backLeftPower = (y - x + rx) / denominator;
//            double frontRightPower = (y - x - rx) / denominator;
//            double backRightPower = (y + x - rx) / denominator;

            // different drive train
            double frontLeftPower = (y + x + rx) * .9;
            double frontRightPower = (y - x - rx) * .9;
            double backLeftPower = (y - x + rx) * .9;
            double backRightPower = (y + x - rx) * .9;

            SetDrivetrainMotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        }
    }

    private void specimenHangSlides() {
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlideMotor.setTargetPosition(8473);
        rightSlideMotor.setTargetPosition(-8528);

        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlideMotor.setPower(0.5);
        rightSlideMotor.setPower(0.5);

    }

    private void elbowUp() {
        elbowLeft.setPosition(.9444);
        elbowRight.setPosition(.2938);
    }

    private void elbowDown() {
        shoulderLeft.setPosition(.0428);
        shoulderRight.setPosition(.9522);
        elbowLeft.setPosition(.7194);
        elbowRight.setPosition(.2744);
    }

    private void armStartPosition() {
        shoulderLeft.setPosition(.0711);
        shoulderRight.setPosition(.9256);
        elbowLeft.setPosition(.9289);
        elbowRight.setPosition(.0694);
    }

    private void armBasketPosition() {
        elbowLeft.setPosition(.4067);
        elbowRight.setPosition(.8756);
        shoulderLeft.setPosition(1);
        shoulderRight.setPosition(0);
    }

    private void wristUp() {
        wristLeft.setPosition(.1994);
        wristRight.setPosition(.7956);
    }

    private void wristDown() {
        wristLeft.setPosition(.9317);
        wristRight.setPosition(.0667);
    }

    private void retract() {
//        extendLeft.setPosition(1);
//        extendRight.setPosition(0);
        extendLeft.setPosition(1);
        extendRight.setPosition(0);

    }

    private void extend() {
        extendLeft.setPosition(.2928);
        extendRight.setPosition(.5567);
    }

    private void armPositionSpecimenHang() {
        shoulderLeft.setPosition(.8389);
        shoulderRight.setPosition(0.0339);
        elbowLeft.setPosition(0);
        elbowRight.setPosition(1);
    }

    private void armSpecimenGrab() {
        shoulderLeft.setPosition(0);
        shoulderRight.setPosition(1);
        elbowLeft.setPosition(.4761);
        elbowRight.setPosition(.7462);
    }

    private void fineTuningControls() {
        if (gamepad2.dpad_down) {
            changeServoPositionBy(wristLeft, .002);
            changeServoPositionBy(wristRight, -.002);
        } else if (gamepad2.dpad_up) {
            changeServoPositionBy(wristLeft, -.002);
            changeServoPositionBy(wristRight, .002);
        }
        if (gamepad2.y) {
            changeServoPositionBy(elbowLeft, .0025);
            changeServoPositionBy(elbowRight, -.0025);
        } else if (gamepad2.a) {
            changeServoPositionBy(elbowLeft, -.0025);
            changeServoPositionBy(elbowRight, .0025);
        }
        if (gamepad1.y) {
            changeServoPositionBy(shoulderLeft, .0025);
            changeServoPositionBy(shoulderRight, -.0025);
        } else if (gamepad1.a) {
            changeServoPositionBy(shoulderLeft, -.0025);
            changeServoPositionBy(shoulderRight,   .0025);
        }


    }

    private void SetDrivetrainMotorPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    private void controlSlides() {
        // SLIDES movement code
        if (gamepad2.left_stick_y < 0 && leftSlideMotor.getCurrentPosition() < 8100 && rightSlideMotor.getCurrentPosition() > -8100) {
            rightSlideMotor.setPower(-gamepad2.left_stick_y * -1);
            leftSlideMotor.setPower(-gamepad2.left_stick_y * 1);
        } else if (gamepad2.left_stick_y > 0 && leftSlideMotor.getCurrentPosition() > 0 && rightSlideMotor.getCurrentPosition() < 0) {
            rightSlideMotor.setPower(-gamepad2.left_stick_y * -1);
            leftSlideMotor.setPower(-gamepad2.left_stick_y * 1);
        } else {
            rightSlideMotor.setPower(0);
            leftSlideMotor.setPower(0);
        }
//        rightSlideMotor.setPower(-gamepad2.left_stick_y * -1);
//        leftSlideMotor.setPower(-gamepad2.left_stick_y * 1);
    }
}
//
//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.Servo;
//
//
//@TeleOp(name = "ASTeleOpFinal1234", group = "TeleOp")
//public class TeleOpFirst extends OpMode {
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
//    private Servo shoulderLeft;
//    private Servo shoulderRight;
//    private Servo elbowLeft;
//    private Servo elbowRight;
//    private Servo clawServo;
//    private Servo extendLeft;
//    private Servo extendRight;
//    private Servo wristLeft;
//    private Servo wristRight;
//    private CRServo spinTakeLeft;
//    private CRServo spinTakeRight;
//
//
//    @Override
//    public void init() {
//        // Drivetrain motors
//        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
//        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
//        backRight = hardwareMap.get(DcMotor.class, "backRight");
//
//
//        // Reverse left motors for proper directionality
//        frontLeft.setDirection(DcMotor.Direction.REVERSE);
//        backLeft.setDirection(DcMotor.Direction.REVERSE);
//
//
//        // left and right slide motors initialized
//        leftSlideMotor = hardwareMap.get(DcMotor.class, "leftSlideMotor");
//        rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");
//        // Reset encoders for the slide motors
//        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        // Set them to run using encoders after reset
//        leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        elbowLeft = hardwareMap.get(Servo.class, "elbowLeft");
//        elbowRight = hardwareMap.get(Servo.class, "elbowRight");
//
//        shoulderLeft = hardwareMap.get(Servo.class, "shoulderLeft");
//        shoulderRight = hardwareMap.get(Servo.class, "shoulderRight");
//
//        spinTakeLeft = hardwareMap.get(CRServo.class, "spinTakeLeft");
//        spinTakeRight = hardwareMap.get(CRServo.class, "spinTakeRight");
//
//        extendLeft = hardwareMap.get(Servo.class, "leftExtend");
//        extendRight = hardwareMap.get(Servo.class, "rightExtend");
//
//        wristRight = hardwareMap.get(Servo.class, "wristRight");
//        wristLeft = hardwareMap.get(Servo.class, "wristLeft");
//
//        clawServo = hardwareMap.get(Servo.class, "clawServo");
//    }
//
//    @Override
//    public void start() {
//        shoulderLeft.setPosition(0);
//        shoulderRight.setPosition(1);
//        elbowLeft.setPosition(9939);
//        elbowRight.setPosition(.2667);
//
//        extendLeft.setPosition(1);
//        extendRight.setPosition(0);
//
//        elbowLeft.setPosition(.9822);
//        elbowRight.setPosition(.26);
//
//        wristLeft.setPosition(0);
//        wristRight.setPosition(1);
//
//        clawServo.setPosition(.6794);
//    }
//
//
//    @Override
//    public void loop() {
//        controlDrivetrain();
//        controlSlides();
//        if (gamepad2.dpad_up) {
//            changeServoPositionBy(extendLeft, -.001);
//            changeServoPositionBy(extendRight, .001);
//        } else if (gamepad2.dpad_down) {
//            changeServoPositionBy(extendLeft, .001);
//            changeServoPositionBy(extendRight, -.001);
//        }
//
//        if (gamepad1.b) {
//            shoulderLeft.setPosition(.7767);
//            shoulderRight.setPosition(.2217);
//            elbowLeft.setPosition(.2289);
//            elbowRight.setPosition(1);
//        }
//
//        if (gamepad2.y) {
//            changeServoPositionBy(elbowLeft, .0025);
//            changeServoPositionBy(elbowRight, -.0025);
//        } else if (gamepad2.a) {
//            changeServoPositionBy(elbowLeft, -.0025);
//            changeServoPositionBy(elbowRight, .0025);
//        }
//
//        if (gamepad1.y) {
//            changeServoPositionBy(shoulderLeft, .0025);
//            changeServoPositionBy(shoulderRight, -.0025);
//        } else if (gamepad1.a) {
//            changeServoPositionBy(shoulderLeft, -.0025);
//            changeServoPositionBy(shoulderRight, .0025);
//        }
//
//        if (gamepad2.left_bumper) { // close claw
//            // changeServoPositionBy(clawServo, .0025);
//            clawServo.setPosition(1);
//        } else if (gamepad2.right_bumper) { // open claw
//            clawServo.setPosition(.6794);
//        }
//
//        if (gamepad2.right_trigger > 0) { // gets wrist up
//            wristLeft.setPosition(0);
//            wristRight.setPosition(1);
//        } else if(gamepad2.left_trigger > 0) { // turn wrist down
//            wristLeft.setPosition(.9317);
//            wristRight.setPosition(.0667);
//        }
//
//        if (gamepad2.dpad_left) { // retract extend
//            try {
//                // elbow goes up
////                elbowLeft.setPosition(.9822);
////                elbowRight.setPosition(.26);
//                elbowLeft.setPosition(.8867);
//                elbowRight.setPosition(.2211);
//
//                // retracts extend
//                extendLeft.setPosition(1);
//                extendRight.setPosition(0);
//                Thread.sleep(1000); // 200 milliseconds delay
//
//                // elbow down
//                elbowLeft.setPosition(9756);
//                elbowRight.setPosition(.2844);
//                Thread.sleep(200); // 200 milliseconds delay
//                // Uncomment if you want small incremental changes with delay
//                // changeServoPositionBy(extendLeft, .001);
//                // changeServoPositionBy(extendRight, -.001);
//            } catch (InterruptedException e) {
//                // Handle the interrupted exception
//                Thread.currentThread().interrupt(); // Re-interrupt the thread
//            }
//        } else if (gamepad2.dpad_right) { // extend extend servos
//            extendLeft.setPosition(.2928);
//            extendRight.setPosition(.7033);
//        }
//
//        if (gamepad1.right_bumper) { // intake
//            spinTakeRight.setPower(1);
//            spinTakeLeft.setPower(-1);
//        } else if (gamepad1.left_bumper) { // ex take
//            spinTakeRight.setPower(-1);
//            spinTakeLeft.setPower(1);
//        } else {
//            spinTakeRight.setPower(0);
//            spinTakeLeft.setPower(0);
//        }
//
//        if (gamepad2.x) {
//            elbowLeft.setPosition(.4489);
//            elbowRight.setPosition(.8389);
//            shoulderLeft.setPosition(.89);
//            shoulderRight.setPosition(0);
//        } else if (gamepad2.b) {
//            shoulderLeft.setPosition(0);
//            shoulderRight.setPosition(1);
//            elbowLeft.setPosition(.9822);
//            elbowRight.setPosition(.26);
////            elbowLeft.setPosition(.8867);
////            elbowRight.setPosition(.2211);
////            elbowLeft.setPosition(.9594);
////            elbowRight.setPosition(.3033);
//        }
//
//        updateAllTelemetry();
//    }
//
//    private void changeServoPositionBy(Servo servo, double delta) {
//        servo.setPosition(servo.getPosition() + delta);
//    }
//
//
//    private void updateAllTelemetry() {
//        telemetry.addData("Left elbow servo arm: ", elbowLeft.getPosition());
//        telemetry.addData("Right elbow servo arm: ", elbowRight.getPosition());
//        telemetry.addData("Left extend: ", extendLeft.getPosition());
//        telemetry.addData("Right extend: ", extendRight.getPosition());
//        telemetry.addData("Left slide: ", leftSlideMotor.getCurrentPosition());
//        telemetry.addData("Right slide: ", rightSlideMotor.getCurrentPosition());
//        telemetry.addData("Left wrist: ", wristLeft.getPosition());
//        telemetry.addData("Right wrist: ", wristRight.getPosition());
//        telemetry.addData("Left Shoulder arm: ", shoulderLeft.getPosition());
//        telemetry.addData("Right Shoulder arm: ", shoulderRight.getPosition());
//        telemetry.addData("Claw servo: ", clawServo.getPosition());
//        telemetry.update();
//    }
//
//    private void controlDrivetrain() {
//        double y = -gamepad1.left_stick_y;  // Forward/backward
//        double x = gamepad1.left_stick_x * 1.1;  // Strafe
//        double rx = gamepad1.right_stick_x * .6;  // Rotation
//
//        // Calculate motor powers
////        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
////        double frontLeftPower = (y + x + rx) / denominator;
////        double backLeftPower = (y - x + rx) / denominator;
////        double frontRightPower = (y - x - rx) / denominator;
////        double backRightPower = (y + x - rx) / denominator;
//        // different drive train
//        double frontLeftPower = (y + x + rx) * .9;
//        double frontRightPower = (y - x - rx) * .9;
//        double backLeftPower = (y - x + rx) * .9;
//        double backRightPower = (y + x - rx) * .9;
//
//        backLeft.setPower(backLeftPower);
//        backRight.setPower(backRightPower);
//        frontLeft.setPower(frontLeftPower);
//        frontRight.setPower(frontRightPower);
//    }
//
//
//    private void controlSlides() {
//        // SLIDES movement code
////         if (gamepad2.left_stick_y < 0 && leftSlideMotor.getCurrentPosition() < 8100 && rightSlideMotor.getCurrentPosition() > -8100) {
////             rightSlideMotor.setPower(-gamepad2.left_stick_y * -.8);
////             leftSlideMotor.setPower(-gamepad2.left_stick_y * .8);
////         } else if (gamepad2.left_stick_y > 0 && leftSlideMotor.getCurrentPosition() > 0 && rightSlideMotor.getCurrentPosition() < 0) {
////             rightSlideMotor.setPower(-gamepad2.left_stick_y * -.8);
////             leftSlideMotor.setPower(-gamepad2.left_stick_y * .8);
////         } else {
////             rightSlideMotor.setPower(0);
////             leftSlideMotor.setPower(0);
////         }
//        // Debugging COMMENT LATER ON
//        rightSlideMotor.setPower(-gamepad2.left_stick_y * -.8);
//        leftSlideMotor.setPower(-gamepad2.left_stick_y * .8);
//    }
//}