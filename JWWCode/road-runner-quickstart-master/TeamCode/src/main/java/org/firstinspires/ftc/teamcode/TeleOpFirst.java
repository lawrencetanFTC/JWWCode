//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.Servo;
//
//
//@TeleOp(name = "ASTeleOp", group = "TeleOp")
//public class TeleOpFirst extends OpMode {
//
//    // Define motors for driving
//    private DcMotor frontLeft;
//    private DcMotor frontRight;
//    private DcMotor backLeft;
//    private DcMotor backRight;
//
//
//    // motors for slides
//    private DcMotor leftSlideMotor;
//    private DcMotor rightSlideMotor;
//
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
//
//        elbowLeft = hardwareMap.get(Servo.class, "elbowLeft");
//        elbowRight = hardwareMap.get(Servo.class, "elbowRight");
//
//
//        shoulderLeft = hardwareMap.get(Servo.class, "shoulderLeft");
//        shoulderRight = hardwareMap.get(Servo.class, "shoulderRight");
//
//
//        spinTakeLeft = hardwareMap.get(CRServo.class, "spinTakeLeft");
//        spinTakeRight = hardwareMap.get(CRServo.class, "spinTakeRight");
//
//
//        extendLeft = hardwareMap.get(Servo.class, "leftExtend");
//        extendRight = hardwareMap.get(Servo.class, "rightExtend");
//
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
//        elbowLeft.setPosition(.9594);
//        elbowRight.setPosition(.3033);
//
////        extendLeft.setPosition(1);
////        extendRight.setPosition(0);
//
////        wristLeft.setPosition(0);
////        wristRight.setPosition(1);
//
////        clawServo.setPosition(.6794);
//    }
//
//
//    @Override
//    public void loop() {
//        controlDrivetrain();
//        controlSlides();
//
//        if (gamepad2.y) {
//            changeServoPositionBy(elbowLeft, .0001);
//            changeServoPositionBy(elbowRight, -.0001);
//        } else if (gamepad2.a) {
//            changeServoPositionBy(elbowLeft, -.0001);
//            changeServoPositionBy(elbowRight, .0001);
//        }
//
//        // if (gamepad2.left_bumper) {
//        //     shoulderLeft.setPosition(0);
//        //     shoulderRight.setPosition(1);
//        // } else if (gamepad2.right_bumper) {
//        //     shoulderLeft.setPosition(.89);
//        //     shoulderRight.setPosition(0);
//        // }
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
//            extendLeft.setPosition(1);
//            extendRight.setPosition(0);
//            // changeServoPositionBy(extenLeft, .001);
//            // changeServoPositionBy(extenRight, -.001);
//        } else if (gamepad2.dpad_right) { // extend extend servos
//            // changeServoPositionBy(extenLeft, -.001);
//            // changeServoPositionBy(extenRight, .001);
//            extendLeft.setPosition(.3944);
//            extendRight.setPosition(.6028);
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
//            elbowLeft.setPosition(.9594);
//            elbowRight.setPosition(.3033);
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
//        double rx = gamepad1.right_stick_x;  // Rotation
//
//        // Calculate motor powers
//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//        double frontLeftPower = (y + x + rx) / denominator;
//        double backLeftPower = (y - x + rx) / denominator;
//        double frontRightPower = (y - x - rx) / denominator;
//        double backRightPower = (y + x - rx) / denominator;
//        // different drive train
////        double frontLeftPower = (y + x + rx) * .9;
////        double frontRightPower = (y - x - rx) * .9;
////        double backLeftPower = (y - x + rx) * .9;
////        double backRightPower = (y + x - rx) * .9;
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
//        if (gamepad2.left_stick_y < 0 && leftSlideMotor.getCurrentPosition() < 8100 && rightSlideMotor.getCurrentPosition() > -8100) {
//            rightSlideMotor.setPower(-gamepad2.left_stick_y * -1);
//            leftSlideMotor.setPower(-gamepad2.left_stick_y * 1);
//        } else if (gamepad2.left_stick_y > 0 && leftSlideMotor.getCurrentPosition() > 0 && rightSlideMotor.getCurrentPosition() < 0) {
//            rightSlideMotor.setPower(-gamepad2.left_stick_y * -1);
//            leftSlideMotor.setPower(-gamepad2.left_stick_y * 1);
//        } else {
//            rightSlideMotor.setPower(0);
//            leftSlideMotor.setPower(0);
//        }
//        // Debugging COMMENT LATER ON
//        if (gamepad2.y) {
//            rightSlideMotor.setPower(-gamepad2.left_stick_y * -0.7);
//            leftSlideMotor.setPower(-gamepad2.left_stick_y * 0.7);
//        }
//    }
//    }

// package org.firstinspires.ftc.teamcode;

// import com.qualcomm.robotcore.util.ElapsedTime;
// import com.qualcomm.robotcore.eventloop.opmode.OpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.util.ElapsedTime;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.CRServo;
// import com.qualcomm.robotcore.hardware.Servo;


// @TeleOp(name = "ASTeleOp", group = "TeleOp")
// public class TeleOpFirst extends OpMode {

//     // Define motors for driving
//     private DcMotor frontLeft;
//     private DcMotor frontRight;
//     private DcMotor backLeft;
//     private DcMotor backRight;


//     // motors for slides
//     private DcMotor leftSlideMotor;
//     private DcMotor rightSlideMotor;


//     // Servos
//     private Servo leftShoulderServo;
//     private Servo rightShoulderServo;
//     private Servo leftArmServo;
//     private Servo rightArmServo;
//     private Servo clawServo;
//     private Servo spinTakeExtendLeft;
//     private Servo spinTakeExtendRight;
//     private Servo spinTakeRotationLeft;
//     private Servo spinTakeRotationRight;
//     private CRServo spinTakeLeft;
//     private CRServo spinTakeRight;


//     @Override
//     public void init() {
//         // Drivetrain motors
//         frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
//         frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//         backLeft = hardwareMap.get(DcMotor.class, "backLeft");
//         backRight = hardwareMap.get(DcMotor.class, "backRight");


//         // Reverse left motors for proper directionality
//         frontLeft.setDirection(DcMotor.Direction.REVERSE);
//         backLeft.setDirection(DcMotor.Direction.REVERSE);


//         // left and right slide motors initialized
//         leftSlideMotor = hardwareMap.get(DcMotor.class, "leftSlideMotor");
//         rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");
//         // Reset encoders for the slide motors
//         leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         // Set them to run using encoders after reset
//         leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//         rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


//         leftArmServo = hardwareMap.get(Servo.class, "elbowLeft");
//         rightArmServo = hardwareMap.get(Servo.class, "elbowRight");


//         leftShoulderServo = hardwareMap.get(Servo.class, "shoulderLeft");
//         rightShoulderServo = hardwareMap.get(Servo.class, "shoulderRight");


//         spinTakeLeft = hardwareMap.get(CRServo.class, "spinTakeLeft");
//         spinTakeRight = hardwareMap.get(CRServo.class, "spinTakeRight");


//         spinTakeExtendLeft = hardwareMap.get(Servo.class, "leftExtend");
//         spinTakeExtendRight = hardwareMap.get(Servo.class, "rightExtend");


//         spinTakeRotationRight = hardwareMap.get(Servo.class, "wristRight");
//         spinTakeRotationLeft = hardwareMap.get(Servo.class, "wristLeft");

//         clawServo = hardwareMap.get(Servo.class, "clawServo");
//     }

//     @Override
//     public void start() {
//         leftShoulderServo.setPosition(0);
//         rightShoulderServo.setPosition(1);
//         leftArmServo.setPosition(.9594);
//         rightArmServo.setPosition(.3033);
//     }


//     @Override
//     public void loop() {


//         // ---- Drivetrain Control ----
//         controlDrivetrain();
//         // // ------ Manual Controls ------
//         // controlSpinTakeManual();
//         // // ------ Control all claw things -----
//         // controlClaw();
//         // // ----- Control slides -------
//         // controlSlides();
//         // // ----- update values of servos and motors -----
//         // updateAllTelemetry();


//         // if (gamepad2.y) {
//         //     leftArmServo.setPosition(1);
//         //     rightArmServo.setPosition(.2733);
//         // } else if (gamepad2.a) {
//         //     leftArmServo.setPosition(.2894);
//         //     rightArmServo.setPosition(1);
//         // }

//         if (gamepad2.y) {
//             changeServoPositionBy(leftArmServo, .0001);
//             changeServoPositionBy(rightArmServo, -.0001);
//         } else if (gamepad2.a) {
//             changeServoPositionBy(leftArmServo, -.0001);
//             changeServoPositionBy(rightArmServo, .0001);
//         }

//         // telemetry.addData("Left elbow: ", leftArmServo.getPosition());
//         // telemetry.addData("Right elbow: ", rightArmServo.getPosition());

//         // if (gamepad2.left_bumper) {
//         //     leftShoulderServo.setPosition(0);
//         //     rightShoulderServo.setPosition(1);
//         // } else if (gamepad2.right_bumper) {
//         //     leftShoulderServo.setPosition(.89);
//         //     rightShoulderServo.setPosition(0);
//         // }

//         if (gamepad2.left_bumper) {
//             // changeServoPositionBy(clawServo, .0025);
//             clawServo.setPosition(1);
//         } else if (gamepad2.right_bumper) {
//             clawServo.setPosition(.6794);
//         }
//         telemetry.addData("claw : ", clawServo.getPosition());

//         if (gamepad2.right_trigger > 0) {
//             spinTakeRotationLeft.setPosition(0);
//             spinTakeRotationRight.setPosition(1);
//         } else if(gamepad2.left_trigger > 0) {
//             spinTakeRotationLeft.setPosition(.9317);
//             spinTakeRotationRight.setPosition(.0667);
//         }
//         telemetry.addData("Left wrist: ", spinTakeRotationLeft.getPosition());
//         telemetry.addData("Right wrist: ", spinTakeRotationRight.getPosition());

//         if (gamepad2.dpad_left) {
//             spinTakeExtendLeft.setPosition(1);
//             spinTakeExtendRight.setPosition(0);
//             // changeServoPositionBy(spinTakeExtendLeft, .001);
//             // changeServoPositionBy(spinTakeExtendRight, -.001);
//         } else if (gamepad2.dpad_right) {
//             // changeServoPositionBy(spinTakeExtendLeft, -.001);
//             // changeServoPositionBy(spinTakeExtendRight, .001);
//             spinTakeExtendLeft.setPosition(.3944);
//             spinTakeExtendRight.setPosition(.6028);
//         }

//         if (gamepad1.right_bumper) {
//             spinTakeRight.setPower(1);
//             spinTakeLeft.setPower(-1);
//         } else if (gamepad1.left_bumper) {
//             spinTakeRight.setPower(-1);
//             spinTakeLeft.setPower(1);
//         } else {
//             spinTakeRight.setPower(0);
//             spinTakeLeft.setPower(0);
//         }


//         if (gamepad2.x) {
//             // timer.reset();
//             leftArmServo.setPosition(.2894);
//             rightArmServo.setPosition(1);
//             leftShoulderServo.setPosition(.89);
//             rightShoulderServo.setPosition(0);
//         } else if (gamepad2.b) {
//             leftShoulderServo.setPosition(0);
//             rightShoulderServo.setPosition(1);
//             leftArmServo.setPosition(.9594);
//         rightArmServo.setPosition(.3033);
//         }

//         if (gamepad1.y) {
//             spinTakeExtendLeft.setPosition(.8322);
//             spinTakeExtendRight.setPosition(.3028);
//         }



//         telemetry.addData("elbow servo arm: ", leftArmServo.getPosition());
//         telemetry.addData("elbow servo arm: ", rightArmServo.getPosition());
//         telemetry.addData("Left extend: ", spinTakeExtendLeft.getPosition());
//         telemetry.addData("Right servo: ", spinTakeExtendRight.getPosition());
//         telemetry.addData("Left slide: ", leftSlideMotor.getCurrentPosition());
//         telemetry.addData("Right right: ", rightSlideMotor.getCurrentPosition());
//         telemetry.addData("Left wrist: ", spinTakeRotationLeft.getPosition());
//         telemetry.addData("Right wrist: ", spinTakeRotationRight.getPosition());
//         telemetry.addData("Left Shoulder arm: ", leftShoulderServo.getPosition());
//         telemetry.addData("Right Shoulder arm: ", rightShoulderServo.getPosition());
//         telemetry.update();

//         controlSlides();

//     }


//     private void changeServoPositionBy(Servo servo, double delta) {
//         servo.setPosition(servo.getPosition() + delta);
//     }


//     private void updateAllTelemetry() {
//         // telemetry.addData("SpinTake rotation servo left: ", spinTakeRotationLeft.getPosition());
//         // telemetry.addData("SpinTake rotation servo right: ", spinTakeRotationRight.getPosition());
//         // telemetry.addData("Left servo arm: ", leftArmServo.getPosition());
//         // telemetry.addData("Right servo arm: ", rightArmServo.getPosition());
//         // telemetry.addData("Left Shoulder arm: ", leftShoulderServo.getPosition());
//         // telemetry.addData("Right Shoulder arm: ", rightShoulderServo.getPosition());
//         // telemetry.addData("Left slide: ", leftSlideMotor.getCurrentPosition());
//         // telemetry.addData("Right right: ", rightSlideMotor.getCurrentPosition());
//         telemetry.update();
//     }


//     private void controlDrivetrain() {
//         double y = -gamepad1.left_stick_y;  // Forward/backward
//         double x = gamepad1.left_stick_x * 1.1;  // Strafe
//         double rx = gamepad1.right_stick_x;  // Rotation


//         // Calculate motor powers
//         double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//         double frontLeftPower = (y + x + rx) / denominator;
//         double backLeftPower = (y - x + rx) / denominator;
//         double frontRightPower = (y - x - rx) / denominator;
//         double backRightPower = (y + x - rx) / denominator;
//         // different drive train
// //        double frontLeftPower = (y + x + rx) * .9;
// //        double frontRightPower = (y - x - rx) * .9;
// //        double backLeftPower = (y - x + rx) * .9;
// //        double backRightPower = (y + x - rx) * .9;


//         // Set motor powers
//         backLeft.setPower(backLeftPower);
//         backRight.setPower(backRightPower);
//         frontLeft.setPower(frontLeftPower);
//         frontRight.setPower(frontRightPower);
//     }


//     private void controlSlides() {
//         // SLIDES movement code
//         if (gamepad2.left_stick_y < 0 && leftSlideMotor.getCurrentPosition() < 8100 && rightSlideMotor.getCurrentPosition() > -8100) {
//             rightSlideMotor.setPower(-gamepad2.left_stick_y * -1);
//             leftSlideMotor.setPower(-gamepad2.left_stick_y * 1);
//         } else if (gamepad2.left_stick_y > 0 && leftSlideMotor.getCurrentPosition() > 0 && rightSlideMotor.getCurrentPosition() < 0) {
//             rightSlideMotor.setPower(-gamepad2.left_stick_y * -1);
//             leftSlideMotor.setPower(-gamepad2.left_stick_y * 1);
//         } else {
//             rightSlideMotor.setPower(0);
//             leftSlideMotor.setPower(0);
//         }
//         // Debugging COMMENT LATER ON
//         if (gamepad2.y) {
//             rightSlideMotor.setPower(-gamepad2.left_stick_y * -0.7);
//             leftSlideMotor.setPower(-gamepad2.left_stick_y * 0.7);
//         }
//     }


//     private void controlSpinTakeManual() {
//         // SpinTake code
//         if (gamepad1.right_bumper) {
//             spinTakeRight.setPower(1);
//             spinTakeLeft.setPower(-1);
//         } else if (gamepad1.left_bumper) {
//             spinTakeRight.setPower(-1);
//             spinTakeLeft.setPower(1);
//         } else {
//             spinTakeRight.setPower(0);
//             spinTakeLeft.setPower(0);
//         }
//         // Rotation of SpinTake
//         if (gamepad2.dpad_left) {
//             changeServoPositionBy(spinTakeRotationLeft, .01);
//             changeServoPositionBy(spinTakeRotationRight, -.01);
//         } else if(gamepad2.dpad_right) {
//             changeServoPositionBy(spinTakeRotationLeft, -.01);
//             changeServoPositionBy(spinTakeRotationRight, .01);
//         }


//         // Extend for spinTake
//         if (gamepad2.right_trigger > 0) {
//             changeServoPositionBy(spinTakeExtendRight, .01);
//             changeServoPositionBy(spinTakeExtendLeft, -.01);
//         } else if (gamepad1.left_trigger > 0) {
//             changeServoPositionBy(spinTakeExtendRight, -.01);
//             changeServoPositionBy(spinTakeExtendLeft, .01);
//         }
//     }


//     private void controlClaw() {
//         // CLAW servo
//         if(gamepad2.right_bumper){
//             changeServoPositionBy(clawServo, .001);
//         } else if(gamepad2.left_bumper){
//             changeServoPositionBy(clawServo, -.001);
//         }


//         // ARM
//         if (gamepad2.left_trigger > 0) {
//             changeServoPositionBy(leftArmServo, .01);
//             changeServoPositionBy(leftArmServo, -.01);
//         } else if (gamepad2.right_trigger > 0) {
//             changeServoPositionBy(leftArmServo, -.01);
//             changeServoPositionBy(leftArmServo, .01);
//         }


//         // ARM shoulder servo thing
//         if (gamepad2.left_bumper) {
//             changeServoPositionBy(leftShoulderServo, .01);
//             changeServoPositionBy(rightShoulderServo, -.01);
//         } else if (gamepad2.right_bumper) {
//             changeServoPositionBy(leftShoulderServo, -.01);
//             changeServoPositionBy(rightShoulderServo, .01);
//         }
//     }


// }
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
//@TeleOp(name = "ASTeleOpFinal", group = "TeleOp")
//public class TeleOpFirst extends OpMode {
//
//    // Define motors for driving
//    private DcMotor frontLeft;
//    private DcMotor frontRight;
//    private DcMotor backLeft;
//    private DcMotor backRight;
//
//
//    // motors for slides
//    private DcMotor leftSlideMotor;
//    private DcMotor rightSlideMotor;
//
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
//
//        elbowLeft = hardwareMap.get(Servo.class, "elbowLeft");
//        elbowRight = hardwareMap.get(Servo.class, "elbowRight");
//
//
//        shoulderLeft = hardwareMap.get(Servo.class, "shoulderLeft");
//        shoulderRight = hardwareMap.get(Servo.class, "shoulderRight");
//
//
//        spinTakeLeft = hardwareMap.get(CRServo.class, "spinTakeLeft");
//        spinTakeRight = hardwareMap.get(CRServo.class, "spinTakeRight");
//
//
//        extendLeft = hardwareMap.get(Servo.class, "leftExtend");
//        extendRight = hardwareMap.get(Servo.class, "rightExtend");
//
//
//        wristRight = hardwareMap.get(Servo.class, "wristRight");
//        wristLeft = hardwareMap.get(Servo.class, "wristLeft");
//
//        clawServo = hardwareMap.get(Servo.class, "clawServo");
//    }
//
//    @Override
//    public void start() {
////        shoulderLeft.setPosition(0);
////        shoulderRight.setPosition(1);
////        elbowLeft.setPosition(.9594);
////        elbowRight.setPosition(.3033);
//
////        extendLeft.setPosition(1);
////        extendRight.setPosition(0);
//
////        wristLeft.setPosition(0);
////        wristRight.setPosition(1);
//
////        clawServo.setPosition(.6794);
//    }
//
//
//    @Override
//    public void loop() {
//        controlDrivetrain();
//        controlSlides();
//
//        if (gamepad2.y) {
//            changeServoPositionBy(elbowLeft, .0001);
//            changeServoPositionBy(elbowRight, -.0001);
//        } else if (gamepad2.a) {
//            changeServoPositionBy(elbowLeft, -.0001);
//            changeServoPositionBy(elbowRight, .0001);
//        }
//
//        // if (gamepad2.left_bumper) {
//        //     shoulderLeft.setPosition(0);
//        //     shoulderRight.setPosition(1);
//        // } else if (gamepad2.right_bumper) {
//        //     shoulderLeft.setPosition(.89);
//        //     shoulderRight.setPosition(0);
//        // }
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
//            extendLeft.setPosition(1);
//            extendRight.setPosition(0);
//            // changeServoPositionBy(extenLeft, .001);
//            // changeServoPositionBy(extenRight, -.001);
//        } else if (gamepad2.dpad_right) { // extend extend servos
//            // changeServoPositionBy(extenLeft, -.001);
//            // changeServoPositionBy(extenRight, .001);
//            extendLeft.setPosition(.3944);
//            extendRight.setPosition(.6028);
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
//            elbowLeft.setPosition(.9594);
//            elbowRight.setPosition(.3033);
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
//        double rx = gamepad1.right_stick_x;  // Rotation
//
//        // Calculate motor powers
//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//        double frontLeftPower = (y + x + rx) / denominator;
//        double backLeftPower = (y - x + rx) / denominator;
//        double frontRightPower = (y - x - rx) / denominator;
//        double backRightPower = (y + x - rx) / denominator;
//        // different drive train
////        double frontLeftPower = (y + x + rx) * .9;
////        double frontRightPower = (y - x - rx) * .9;
////        double backLeftPower = (y - x + rx) * .9;
////        double backRightPower = (y + x - rx) * .9;
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
////        if (gamepad2.left_stick_y < 0 && leftSlideMotor.getCurrentPosition() < 8100 && rightSlideMotor.getCurrentPosition() > -8100) {
////            rightSlideMotor.setPower(-gamepad2.left_stick_y * -.8);
////            leftSlideMotor.setPower(-gamepad2.left_stick_y * .8);
////        } else if (gamepad2.left_stick_y > 0 && leftSlideMotor.getCurrentPosition() > 0 && rightSlideMotor.getCurrentPosition() < 0) {
////            rightSlideMotor.setPower(-gamepad2.left_stick_y * -.8);
////            leftSlideMotor.setPower(-gamepad2.left_stick_y * .8);
////        } else {
////            rightSlideMotor.setPower(0);
////            leftSlideMotor.setPower(0);
////        }
//        // Debugging COMMENT LATER ON
//         rightSlideMotor.setPower(-gamepad2.right_stick_y * -.8);
//         leftSlideMotor.setPower(-gamepad2.right_stick_y * .8);
//    }
//}

// package org.firstinspires.ftc.teamcode;

// import com.qualcomm.robotcore.util.ElapsedTime;
// import com.qualcomm.robotcore.eventloop.opmode.OpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.util.ElapsedTime;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.CRServo;
// import com.qualcomm.robotcore.hardware.Servo;


// @TeleOp(name = "ASTeleOp", group = "TeleOp")
// public class TeleOpFirst extends OpMode {

//     // Define motors for driving
//     private DcMotor frontLeft;
//     private DcMotor frontRight;
//     private DcMotor backLeft;
//     private DcMotor backRight;


//     // motors for slides
//     private DcMotor leftSlideMotor;
//     private DcMotor rightSlideMotor;


//     // Servos
//     private Servo leftShoulderServo;
//     private Servo rightShoulderServo;
//     private Servo leftArmServo;
//     private Servo rightArmServo;
//     private Servo clawServo;
//     private Servo spinTakeExtendLeft;
//     private Servo spinTakeExtendRight;
//     private Servo spinTakeRotationLeft;
//     private Servo spinTakeRotationRight;
//     private CRServo spinTakeLeft;
//     private CRServo spinTakeRight;


//     @Override
//     public void init() {
//         // Drivetrain motors
//         frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
//         frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//         backLeft = hardwareMap.get(DcMotor.class, "backLeft");
//         backRight = hardwareMap.get(DcMotor.class, "backRight");


//         // Reverse left motors for proper directionality
//         frontLeft.setDirection(DcMotor.Direction.REVERSE);
//         backLeft.setDirection(DcMotor.Direction.REVERSE);


//         // left and right slide motors initialized
//         leftSlideMotor = hardwareMap.get(DcMotor.class, "leftSlideMotor");
//         rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");
//         // Reset encoders for the slide motors
//         leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         // Set them to run using encoders after reset
//         leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//         rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


//         leftArmServo = hardwareMap.get(Servo.class, "elbowLeft");
//         rightArmServo = hardwareMap.get(Servo.class, "elbowRight");


//         leftShoulderServo = hardwareMap.get(Servo.class, "shoulderLeft");
//         rightShoulderServo = hardwareMap.get(Servo.class, "shoulderRight");


//         spinTakeLeft = hardwareMap.get(CRServo.class, "spinTakeLeft");
//         spinTakeRight = hardwareMap.get(CRServo.class, "spinTakeRight");


//         spinTakeExtendLeft = hardwareMap.get(Servo.class, "leftExtend");
//         spinTakeExtendRight = hardwareMap.get(Servo.class, "rightExtend");


//         spinTakeRotationRight = hardwareMap.get(Servo.class, "wristRight");
//         spinTakeRotationLeft = hardwareMap.get(Servo.class, "wristLeft");

//         clawServo = hardwareMap.get(Servo.class, "clawServo");
//     }

//     @Override
//     public void start() {
//         leftShoulderServo.setPosition(0);
//         rightShoulderServo.setPosition(1);
//         leftArmServo.setPosition(.9594);
//         rightArmServo.setPosition(.3033);
//     }


//     @Override
//     public void loop() {


//         // ---- Drivetrain Control ----
//         controlDrivetrain();
//         // // ------ Manual Controls ------
//         // controlSpinTakeManual();
//         // // ------ Control all claw things -----
//         // controlClaw();
//         // // ----- Control slides -------
//         // controlSlides();
//         // // ----- update values of servos and motors -----
//         // updateAllTelemetry();


//         // if (gamepad2.y) {
//         //     leftArmServo.setPosition(1);
//         //     rightArmServo.setPosition(.2733);
//         // } else if (gamepad2.a) {
//         //     leftArmServo.setPosition(.2894);
//         //     rightArmServo.setPosition(1);
//         // }

//         if (gamepad2.y) {
//             changeServoPositionBy(leftArmServo, .0001);
//             changeServoPositionBy(rightArmServo, -.0001);
//         } else if (gamepad2.a) {
//             changeServoPositionBy(leftArmServo, -.0001);
//             changeServoPositionBy(rightArmServo, .0001);
//         }

//         // telemetry.addData("Left elbow: ", leftArmServo.getPosition());
//         // telemetry.addData("Right elbow: ", rightArmServo.getPosition());

//         // if (gamepad2.left_bumper) {
//         //     leftShoulderServo.setPosition(0);
//         //     rightShoulderServo.setPosition(1);
//         // } else if (gamepad2.right_bumper) {
//         //     leftShoulderServo.setPosition(.89);
//         //     rightShoulderServo.setPosition(0);
//         // }

//         if (gamepad2.left_bumper) {
//             // changeServoPositionBy(clawServo, .0025);
//             clawServo.setPosition(1);
//         } else if (gamepad2.right_bumper) {
//             clawServo.setPosition(.6794);
//         }
//         telemetry.addData("claw : ", clawServo.getPosition());

//         if (gamepad2.right_trigger > 0) {
//             spinTakeRotationLeft.setPosition(0);
//             spinTakeRotationRight.setPosition(1);
//         } else if(gamepad2.left_trigger > 0) {
//             spinTakeRotationLeft.setPosition(.9317);
//             spinTakeRotationRight.setPosition(.0667);
//         }
//         telemetry.addData("Left wrist: ", spinTakeRotationLeft.getPosition());
//         telemetry.addData("Right wrist: ", spinTakeRotationRight.getPosition());

//         if (gamepad2.dpad_left) {
//             spinTakeExtendLeft.setPosition(1);
//             spinTakeExtendRight.setPosition(0);
//             // changeServoPositionBy(spinTakeExtendLeft, .001);
//             // changeServoPositionBy(spinTakeExtendRight, -.001);
//         } else if (gamepad2.dpad_right) {
//             // changeServoPositionBy(spinTakeExtendLeft, -.001);
//             // changeServoPositionBy(spinTakeExtendRight, .001);
//             spinTakeExtendLeft.setPosition(.3944);
//             spinTakeExtendRight.setPosition(.6028);
//         }

//         if (gamepad1.right_bumper) {
//             spinTakeRight.setPower(1);
//             spinTakeLeft.setPower(-1);
//         } else if (gamepad1.left_bumper) {
//             spinTakeRight.setPower(-1);
//             spinTakeLeft.setPower(1);
//         } else {
//             spinTakeRight.setPower(0);
//             spinTakeLeft.setPower(0);
//         }


//         if (gamepad2.x) {
//             // timer.reset();
//             leftArmServo.setPosition(.2894);
//             rightArmServo.setPosition(1);
//             leftShoulderServo.setPosition(.89);
//             rightShoulderServo.setPosition(0);
//         } else if (gamepad2.b) {
//             leftShoulderServo.setPosition(0);
//             rightShoulderServo.setPosition(1);
//             leftArmServo.setPosition(.9594);
//         rightArmServo.setPosition(.3033);
//         }

//         if (gamepad1.y) {
//             spinTakeExtendLeft.setPosition(.8322);
//             spinTakeExtendRight.setPosition(.3028);
//         }



//         telemetry.addData("elbow servo arm: ", leftArmServo.getPosition());
//         telemetry.addData("elbow servo arm: ", rightArmServo.getPosition());
//         telemetry.addData("Left extend: ", spinTakeExtendLeft.getPosition());
//         telemetry.addData("Right servo: ", spinTakeExtendRight.getPosition());
//         telemetry.addData("Left slide: ", leftSlideMotor.getCurrentPosition());
//         telemetry.addData("Right right: ", rightSlideMotor.getCurrentPosition());
//         telemetry.addData("Left wrist: ", spinTakeRotationLeft.getPosition());
//         telemetry.addData("Right wrist: ", spinTakeRotationRight.getPosition());
//         telemetry.addData("Left Shoulder arm: ", leftShoulderServo.getPosition());
//         telemetry.addData("Right Shoulder arm: ", rightShoulderServo.getPosition());
//         telemetry.update();

//         controlSlides();

//     }


//     private void changeServoPositionBy(Servo servo, double delta) {
//         servo.setPosition(servo.getPosition() + delta);
//     }


//     private void updateAllTelemetry() {
//         // telemetry.addData("SpinTake rotation servo left: ", spinTakeRotationLeft.getPosition());
//         // telemetry.addData("SpinTake rotation servo right: ", spinTakeRotationRight.getPosition());
//         // telemetry.addData("Left servo arm: ", leftArmServo.getPosition());
//         // telemetry.addData("Right servo arm: ", rightArmServo.getPosition());
//         // telemetry.addData("Left Shoulder arm: ", leftShoulderServo.getPosition());
//         // telemetry.addData("Right Shoulder arm: ", rightShoulderServo.getPosition());
//         // telemetry.addData("Left slide: ", leftSlideMotor.getCurrentPosition());
//         // telemetry.addData("Right right: ", rightSlideMotor.getCurrentPosition());
//         telemetry.update();
//     }


//     private void controlDrivetrain() {
//         double y = -gamepad1.left_stick_y;  // Forward/backward
//         double x = gamepad1.left_stick_x * 1.1;  // Strafe
//         double rx = gamepad1.right_stick_x;  // Rotation


//         // Calculate motor powers
//         double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//         double frontLeftPower = (y + x + rx) / denominator;
//         double backLeftPower = (y - x + rx) / denominator;
//         double frontRightPower = (y - x - rx) / denominator;
//         double backRightPower = (y + x - rx) / denominator;
//         // different drive train
// //        double frontLeftPower = (y + x + rx) * .9;
// //        double frontRightPower = (y - x - rx) * .9;
// //        double backLeftPower = (y - x + rx) * .9;
// //        double backRightPower = (y + x - rx) * .9;


//         // Set motor powers
//         backLeft.setPower(backLeftPower);
//         backRight.setPower(backRightPower);
//         frontLeft.setPower(frontLeftPower);
//         frontRight.setPower(frontRightPower);
//     }


//     private void controlSlides() {
//         // SLIDES movement code
//         if (gamepad2.left_stick_y < 0 && leftSlideMotor.getCurrentPosition() < 8100 && rightSlideMotor.getCurrentPosition() > -8100) {
//             rightSlideMotor.setPower(-gamepad2.left_stick_y * -1);
//             leftSlideMotor.setPower(-gamepad2.left_stick_y * 1);
//         } else if (gamepad2.left_stick_y > 0 && leftSlideMotor.getCurrentPosition() > 0 && rightSlideMotor.getCurrentPosition() < 0) {
//             rightSlideMotor.setPower(-gamepad2.left_stick_y * -1);
//             leftSlideMotor.setPower(-gamepad2.left_stick_y * 1);
//         } else {
//             rightSlideMotor.setPower(0);
//             leftSlideMotor.setPower(0);
//         }
//         // Debugging COMMENT LATER ON
//         if (gamepad2.y) {
//             rightSlideMotor.setPower(-gamepad2.left_stick_y * -0.7);
//             leftSlideMotor.setPower(-gamepad2.left_stick_y * 0.7);
//         }
//     }


//     private void controlSpinTakeManual() {
//         // SpinTake code
//         if (gamepad1.right_bumper) {
//             spinTakeRight.setPower(1);
//             spinTakeLeft.setPower(-1);
//         } else if (gamepad1.left_bumper) {
//             spinTakeRight.setPower(-1);
//             spinTakeLeft.setPower(1);
//         } else {
//             spinTakeRight.setPower(0);
//             spinTakeLeft.setPower(0);
//         }
//         // Rotation of SpinTake
//         if (gamepad2.dpad_left) {
//             changeServoPositionBy(spinTakeRotationLeft, .01);
//             changeServoPositionBy(spinTakeRotationRight, -.01);
//         } else if(gamepad2.dpad_right) {
//             changeServoPositionBy(spinTakeRotationLeft, -.01);
//             changeServoPositionBy(spinTakeRotationRight, .01);
//         }


//         // Extend for spinTake
//         if (gamepad2.right_trigger > 0) {
//             changeServoPositionBy(spinTakeExtendRight, .01);
//             changeServoPositionBy(spinTakeExtendLeft, -.01);
//         } else if (gamepad1.left_trigger > 0) {
//             changeServoPositionBy(spinTakeExtendRight, -.01);
//             changeServoPositionBy(spinTakeExtendLeft, .01);
//         }
//     }


//     private void controlClaw() {
//         // CLAW servo
//         if(gamepad2.right_bumper){
//             changeServoPositionBy(clawServo, .001);
//         } else if(gamepad2.left_bumper){
//             changeServoPositionBy(clawServo, -.001);
//         }


//         // ARM
//         if (gamepad2.left_trigger > 0) {
//             changeServoPositionBy(leftArmServo, .01);
//             changeServoPositionBy(leftArmServo, -.01);
//         } else if (gamepad2.right_trigger > 0) {
//             changeServoPositionBy(leftArmServo, -.01);
//             changeServoPositionBy(leftArmServo, .01);
//         }


//         // ARM shoulder servo thing
//         if (gamepad2.left_bumper) {
//             changeServoPositionBy(leftShoulderServo, .01);
//             changeServoPositionBy(rightShoulderServo, -.01);
//         } else if (gamepad2.right_bumper) {
//             changeServoPositionBy(leftShoulderServo, -.01);
//             changeServoPositionBy(rightShoulderServo, .01);
//         }
//     }


// }

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "ASTeleOp123456789", group = "TeleOp")
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

//        extendLeft.setPosition(1);
//        extendRight.setPosition(0);

//        wristLeft.setPosition(0);
//        wristRight.setPosition(1);

//        clawServo.setPosition(.6794);
    }


    @Override
    public void loop() {
        controlDrivetrain();
        controlSlides();

        if (gamepad2.y) {
            changeServoPositionBy(elbowLeft, .0001);
            changeServoPositionBy(elbowRight, -.0001);
        } else if (gamepad2.a) {
            changeServoPositionBy(elbowLeft, -.0001);
            changeServoPositionBy(elbowRight, .0001);
        }

        // if (gamepad2.left_bumper) {
        //     shoulderLeft.setPosition(0);
        //     shoulderRight.setPosition(1);
        // } else if (gamepad2.right_bumper) {
        //     shoulderLeft.setPosition(.89);
        //     shoulderRight.setPosition(0);
        // }

        if (gamepad2.left_bumper) { // close claw
            // changeServoPositionBy(clawServo, .0025);
            clawServo.setPosition(1);
        } else if (gamepad2.right_bumper) { // open claw
            clawServo.setPosition(.6794);
        }

        if (gamepad2.right_trigger > 0) { // gets wrist up
            wristLeft.setPosition(0);
            wristRight.setPosition(1);
        } else if(gamepad2.left_trigger > 0) { // turn wrist down
            wristLeft.setPosition(.9317);
            wristRight.setPosition(.0667);
        }

        if (gamepad2.dpad_left) { // retract extend
            try {
                // Step 1
                elbowLeft.setPosition(.9183);
                elbowRight.setPosition(.2667);
                extendLeft.setPosition(1);
                extendRight.setPosition(0);
                Thread.sleep(200); // 200 milliseconds delay

                // Step 2
                elbowLeft.setPosition(9756);
                elbowRight.setPosition(.2844);
                Thread.sleep(200); // 200 milliseconds delay

                // Uncomment if you want small incremental changes with delay
                // changeServoPositionBy(extendLeft, .001);
                // changeServoPositionBy(extendRight, -.001);
            } catch (InterruptedException e) {
                // Handle the interrupted exception
                Thread.currentThread().interrupt(); // Re-interrupt the thread
            }
        } else if (gamepad2.dpad_right) { // extend extend servos
            // changeServoPositionBy(extenLeft, -.001);
            // changeServoPositionBy(extenRight, .001);
            extendLeft.setPosition(.3944);
            extendRight.setPosition(.6028);
        }

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

        if (gamepad2.x) {
            elbowLeft.setPosition(.4489);
            elbowRight.setPosition(.8389);
            shoulderLeft.setPosition(.89);
            shoulderRight.setPosition(0);
        } else if (gamepad2.b) {
            shoulderLeft.setPosition(0);
            shoulderRight.setPosition(1);
            elbowLeft.setPosition(.9944);
            elbowRight.setPosition(.2667);
        }

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
        double y = -gamepad1.left_stick_y;  // Forward/backward
        double x = gamepad1.left_stick_x * 1.1;  // Strafe
        double rx = gamepad1.right_stick_x;  // Rotation

        // Calculate motor powers
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
        // different drive train
//        double frontLeftPower = (y + x + rx) * .9;
//        double frontRightPower = (y - x - rx) * .9;
//        double backLeftPower = (y - x + rx) * .9;
//        double backRightPower = (y + x - rx) * .9;

        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
    }


    private void controlSlides() {
        // SLIDES movement code
        // if (gamepad2.left_stick_y < 0 && leftSlideMotor.getCurrentPosition() < 8100 && rightSlideMotor.getCurrentPosition() > -8100) {
        //     rightSlideMotor.setPower(-gamepad2.left_stick_y * -.8);
        //     leftSlideMotor.setPower(-gamepad2.left_stick_y * .8);
        // } else if (gamepad2.left_stick_y > 0 && leftSlideMotor.getCurrentPosition() > 0 && rightSlideMotor.getCurrentPosition() < 0) {
        //     rightSlideMotor.setPower(-gamepad2.left_stick_y * -.8);
        //     leftSlideMotor.setPower(-gamepad2.left_stick_y * .8);
        // } else {
        //     rightSlideMotor.setPower(0);
        //     leftSlideMotor.setPower(0);
        // }
        // Debugging COMMENT LATER ON
        rightSlideMotor.setPower(-gamepad2.right_stick_y * -.8);
        leftSlideMotor.setPower(-gamepad2.right_stick_y * .8);
    }
}
