//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.Servo;
//
//@TeleOp (name = "ASTeleOp", group="TeleOp")
//public class WorkTeleOp extends OpMode {
//    // Define motors for driving
//    private DcMotor frontLeft;
//    private DcMotor frontRight;
//    private DcMotor backLeft;
//    private DcMotor backRight;
//
//    // Servos
//    private Servo clawServo;
//    private CRServo spinTake;            // Continuous servo for intake spin
//    private DcMotor armMotor;            // Motor for spin take arm up and down
//    private CRServo extendServo;
//
//    private DcMotor leftSideMotor;       // Motor for left-side mechanism
//    private DcMotor rightSideMotor;      // Motor for right-side mechanism
//
//    // PID things
//    private final double Kp =  0.5;      // Proportional gain (tune as needed)
//    private final double Ki = 0;      // Integral gain (tune as needed)
//    private final double Kd = 0;      // Derivative gain (tune as needed)
//
//    // PID variables
//    private double targetPosition = 0; // Desired encoder position
//    private double integralSum = 0;
//    private double lastError = 0;
//
//    // Constants for slides
//    private final int MAX_POSITION = 100;       // Maximum encoder ticks (full extension) not correct
//    private final int MIN_POSITION = 0;         // Minimum encoder ticks (fully retracted)
//
//    // constants for claw
//    private final double CLAW_OPEN_POSITION = 0.52;
//    private final double CLAW_CLOSE_POSITION = 0.65;
//
//    @Override
//    public void init() { // for initialization of stuff
//        // setting motors
//        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
//        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
//        backRight = hardwareMap.get(DcMotor.class, "backRight");
//
//        // Reverse the motor on the left side for proper directionality
//        frontLeft.setDirection(DcMotor.Direction.REVERSE);
//        backLeft.setDirection(DcMotor.Direction.REVERSE);
//
//        // servos continuous
//        spinTake = hardwareMap.get(CRServo.class, "spinTake");
//        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
//        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        extendServo = hardwareMap.get(CRServo.class, "extendServo");
//
//        // position servo
//        clawServo = hardwareMap.get(Servo.class, "clawServo");
//
//        // side motors
//        leftSideMotor = hardwareMap.get(DcMotor.class, "leftSideMotor");
//        rightSideMotor = hardwareMap.get(DcMotor.class, "rightSideMotor");
//
//        // arm pid and hold
//        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // if something wrong comment this
//        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // then this one too
//        targetPosition = armMotor.getCurrentPosition();
//    }
//    @Override
//    public void loop() {
//        // controls for drivetrain
////        double y = -gamepad1.left_stick_y;
////        double x = gamepad1.left_stick_x * 1.1;
////        double turn = gamepad1.right_stick_x;
////
////        double frontLeftPower = .90 * (-x - y - turn);
////        double frontRightPower = (y - x - turn) * .90;
////        double backLeftPower = (y - x + turn) * .85;
////        double backRightPower = (y + x - turn) * .75;
//
//        double y = -gamepad1.left_stick_y; // Forward/Backward motion
//        double x = gamepad1.left_stick_x * 1.1; // Strafing (scaled for fine control)
//        double turn = gamepad1.right_stick_x; // Rotation
//
//        // Mecanum wheel calculations
//        double frontLeftPower = (y + x + turn) * .79;  // Corrected logic for front-left wheel
//        double frontRightPower = (y - x - turn) * .79; // Corrected logic for front-right wheel
//        double backLeftPower = (y - x + turn) * .90;   // Corrected logic for back-left wheel
//        double backRightPower = (y + x - turn) * .90;  // Corrected logic for back-right wheel
//
//        // Scale down the powers if any exceeds the range [-1, 1]
//
//
//
//        frontLeft.setPower(frontLeftPower);
//        frontRight.setPower(frontRightPower);
//        backLeft.setPower(backLeftPower);
//        backRight.setPower(backRightPower);
//
//        // ARM ARM
//        // if (gamepad1.left_bumper) {
//        //     targetPosition += .01;  // Adjust upward by an increment
//        // } else if (gamepad1.right_bumper) {
//        //     targetPosition -= .01;  // Adjust downward by an increment
//        // } else {
//        //     targetPosition += 0;
//        // }
//        // pidControl(armMotor, targetPosition);
//
//        // if this doesn't hold then uncomment code above (probably won't work)
//        // if (gamepad1.left_bumper) {
//        //     armMotor.setPower(.4);  // Adjust upward by an increment
//        // } else if (gamepad1.right_bumper) {
//        //     armMotor.setPower(.4);  // Adjust downward by an increment
//        // }
//
//        // extend servo
//        if (gamepad2.dpad_up) {
//            extendServo.setPower(1);
//        } else if (gamepad2.dpad_down) {
//            extendServo.setPower(-1);
//        } else {
//            extendServo.setPower(0);
//        }
//
//        // SpinTake servo
//        if (gamepad1.a) {
//            spinTake.setPower(1);
//        } else if (gamepad1.b) {
//            spinTake.setPower(-1);
//        } else {
//            spinTake.setPower(0);
//        }
//
//
//        // while (opModeIsActive()) { // Adjust target position continuously while pressing D-Pad if (gamepad1.dpad_up) { targetPosition += NORMAL_INCREMENT; // Increase target position } else if (gamepad1.dpad_down) { targetPosition -= NORMAL_INCREMENT; // Decrease target position } // Clamp target position within bounds targetPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, targetPosition)); // Get the current position from encoders double currentPosition = (leftSlideMotor.getCurrentPosition() + rightSlideMotor.getCurrentPosition()) / 2.0; // Calculate PID terms double error = targetPosition - currentPosition; integralSum += error; // Accumulate error for integral term double derivative = error - lastError; // Change in error for derivative term // Compute motor power using PID double power = (Kp * error) + (Ki * integralSum) + (Kd * derivative); // Apply power to both motors leftSlideMotor.setPower(power); rightSlideMotor.setPower(power); // Update last error lastError = error; // Telemetry for debugging telemetry.addData("Target Position", targetPosition); telemetry.addData("Current Position", currentPosition); telemetry.addData("Motor Power", power); telemetry.update(); } } }
//
//        if (gamepad2.right_stick_y > 0) {
//            targetPosition += .001;  // Adjust upward by an increment
//        }
//        else if (gamepad2.right_stick_y < 0) {
//            targetPosition -= .001;  // Adjust downward by an increment
//        }
//
//// getting the current position of the motor
//        double currentPosition = armMotor.getCurrentPosition();
//        double error = targetPosition - currentPosition;
//
//// Proportional term
//        double P = error * Kp;
//
//// Integral term
//        integralSum += error;
//        double I = integralSum * Ki;
//
//// Derivative term
//        double derivative = error - lastError;
//        double D = derivative * Kd;
//
//// Total PID output
//        double powerOutput = P + I + D;
//        armMotor.setPower(powerOutput);
//
//// Update last error
//        lastError = error;
//
//        // claw position servo
//        if (gamepad2.left_bumper) {
//            clawServo.setPosition(CLAW_OPEN_POSITION); // Open claw
//        } else if (gamepad2.right_bumper) {
//            clawServo.setPosition(CLAW_CLOSE_POSITION); // Close claw
//        }
//
//        // slide motors
//        leftSideMotor.setPower(-gamepad2.left_stick_y * 0.8);
//        rightSideMotor.setPower(-gamepad2.left_stick_y * -0.8);
//
//        // Control leftSideMotor with limits (if doesn't work comment this code and uncomment the code above)
//        //     double leftSidePosition = leftSideMotor.getCurrentPosition();
//        //     if ((gamepad2.left_stick_y < 0 && leftSidePosition > MIN_POSITION) ||
//        //             (gamepad2.left_stick_y > 0 && leftSidePosition < MAX_POSITION)) {
//        //         leftSideMotor.setPower(-gamepad2.left_stick_y * 0.8);
//        //     } else {
//        //         leftSideMotor.setPower(0);
//        //     }
//
//        //     // Control rightSideMotor with limits
//        //     double rightSidePosition = rightSideMotor.getCurrentPosition();
//        //     if ((gamepad2.left_stick_y < 0 && rightSidePosition > MIN_POSITION) ||
//        //             (gamepad2.left_stick_y > 0 && rightSidePosition < MAX_POSITION)) {
//        //         rightSideMotor.setPower(gamepad2.left_stick_y * 0.8);
//        //     } else {
//        //         rightSideMotor.setPower(0);
//        //     }
//    }
//
//    private void pidControl(DcMotor motor, double targetPosition) {
//
//
//    }
//}
//
////package org.firstinspires.ftc.teamcode;
////
////import com.qualcomm.robotcore.eventloop.opmode.OpMode;
////import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
////import com.qualcomm.robotcore.hardware.DcMotor;
////import com.qualcomm.robotcore.hardware.CRServo;
////import com.qualcomm.robotcore.hardware.Servo;
////
////
////@TeleOp(name = "ASTeleOp", group = "TeleOp")
////public class TeleOpFirst extends OpMode {
////
////    // Define motors for driving
////    private DcMotor frontLeft;
////    private DcMotor frontRight;
////    private DcMotor backLeft;
////    private DcMotor backRight;
////
////
////    // motors for slides
////    private DcMotor leftSlideMotor;
////    private DcMotor rightSlideMotor;
////
////
////    // Servos
////    private Servo shoulderLeft;
////    private Servo shoulderRight;
////    private Servo elbowLeft;
////    private Servo elbowRight;
////    private Servo clawServo;
////    private Servo extenLeft;
////    private Servo extenRight;
////    private Servo wristLeft;
////    private Servo wristRight;
////    private CRServo spinTakeLeft;
////    private CRServo spinTakeRight;
////
////
////    @Override
////    public void init() {
////        // Drivetrain motors
////        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
////        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
////        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
////        backRight = hardwareMap.get(DcMotor.class, "backRight");
////
////
////        // Reverse left motors for proper directionality
////        frontLeft.setDirection(DcMotor.Direction.REVERSE);
////        backLeft.setDirection(DcMotor.Direction.REVERSE);
////
////
////        // left and right slide motors initialized
////        leftSlideMotor = hardwareMap.get(DcMotor.class, "leftSlideMotor");
////        rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");
////        // Reset encoders for the slide motors
////        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        // Set them to run using encoders after reset
////        leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////
////
////        elbowLeft = hardwareMap.get(Servo.class, "elbowLeft");
////        elbowRight = hardwareMap.get(Servo.class, "elbowRight");
////
////
////        shoulderLeft = hardwareMap.get(Servo.class, "shoulderLeft");
////        shoulderRight = hardwareMap.get(Servo.class, "shoulderRight");
////
////
////        spinTakeLeft = hardwareMap.get(CRServo.class, "spinTakeLeft");
////        spinTakeRight = hardwareMap.get(CRServo.class, "spinTakeRight");
////
////
////        extenLeft = hardwareMap.get(Servo.class, "leftExtend");
////        extenRight = hardwareMap.get(Servo.class, "rightExtend");
////
////
////        wristRight = hardwareMap.get(Servo.class, "wristRight");
////        wristLeft = hardwareMap.get(Servo.class, "wristLeft");
////
////        clawServo = hardwareMap.get(Servo.class, "clawServo");
////    }
////
////    @Override
////    public void start() {
////        shoulderLeft.setPosition(0);
////        shoulderRight.setPosition(1);
////        elbowLeft.setPosition(.9594);
////        elbowRight.setPosition(.3033);
////
//////        extenLeft.setPosition(1);
//////        extenRight.setPosition(0);
////
//////        wristLeft.setPosition(0);
//////        wristRight.setPosition(1);
////
//////        clawServo.setPosition(.6794);
////    }
////
////
////    @Override
////    public void loop() {
////        controlDrivetrain();
////        controlSlides();
////
////        if (gamepad2.y) {
////            changeServoPositionBy(elbowLeft, .0001);
////            changeServoPositionBy(elbowRight, -.0001);
////        } else if (gamepad2.a) {
////            changeServoPositionBy(elbowLeft, -.0001);
////            changeServoPositionBy(elbowRight, .0001);
////        }
////
////        // if (gamepad2.left_bumper) {
////        //     shoulderLeft.setPosition(0);
////        //     shoulderRight.setPosition(1);
////        // } else if (gamepad2.right_bumper) {
////        //     shoulderLeft.setPosition(.89);
////        //     shoulderRight.setPosition(0);
////        // }
////
////        if (gamepad2.left_bumper) { // close claw
////            // changeServoPositionBy(clawServo, .0025);
////            clawServo.setPosition(1);
////        } else if (gamepad2.right_bumper) { // open claw
////            clawServo.setPosition(.6794);
////        }
////
////        if (gamepad2.right_trigger > 0) { // gets wrist up
////            wristLeft.setPosition(0);
////            wristRight.setPosition(1);
////        } else if(gamepad2.left_trigger > 0) { // turn wrist down
////            wristLeft.setPosition(.9317);
////            wristRight.setPosition(.0667);
////        }
////
////        if (gamepad2.dpad_left) { // retract extend
////            extenLeft.setPosition(1);
////            extenRight.setPosition(0);
////            // changeServoPositionBy(extenLeft, .001);
////            // changeServoPositionBy(extenRight, -.001);
////        } else if (gamepad2.dpad_right) { // extend extend servos
////            // changeServoPositionBy(extenLeft, -.001);
////            // changeServoPositionBy(extenRight, .001);
////            extenLeft.setPosition(.3944);
////            extenRight.setPosition(.6028);
////        }
////
////        if (gamepad1.right_bumper) { // intake
////            spinTakeRight.setPower(1);
////            spinTakeLeft.setPower(-1);
////        } else if (gamepad1.left_bumper) { // ex take
////            spinTakeRight.setPower(-1);
////            spinTakeLeft.setPower(1);
////        } else {
////            spinTakeRight.setPower(0);
////            spinTakeLeft.setPower(0);
////        }
////
////        if (gamepad2.x) {
////            elbowLeft.setPosition(.4489);
////            elbowRight.setPosition(.8389);
////            shoulderLeft.setPosition(.89);
////            shoulderRight.setPosition(0);
////        } else if (gamepad2.b) {
////            shoulderLeft.setPosition(0);
////            shoulderRight.setPosition(1);
////            elbowLeft.setPosition(.9594);
////            elbowRight.setPosition(.3033);
////        }
////
////        updateAllTelemetry();
////    }
////
////    private void changeServoPositionBy(Servo servo, double delta) {
////        servo.setPosition(servo.getPosition() + delta);
////    }
////
////
////    private void updateAllTelemetry() {
////        telemetry.addData("Left elbow servo arm: ", elbowLeft.getPosition());
////        telemetry.addData("Right elbow servo arm: ", elbowRight.getPosition());
////        telemetry.addData("Left extend: ", extenLeft.getPosition());
////        telemetry.addData("Right extend: ", extenRight.getPosition());
////        telemetry.addData("Left slide: ", leftSlideMotor.getCurrentPosition());
////        telemetry.addData("Right slide: ", rightSlideMotor.getCurrentPosition());
////        telemetry.addData("Left wrist: ", wristLeft.getPosition());
////        telemetry.addData("Right wrist: ", wristRight.getPosition());
////        telemetry.addData("Left Shoulder arm: ", shoulderLeft.getPosition());
////        telemetry.addData("Right Shoulder arm: ", shoulderRight.getPosition());
////        telemetry.addData("Claw servo: ", clawServo.getPosition());
////        telemetry.update();
////    }
////
////    private void controlDrivetrain() {
////        double y = -gamepad1.left_stick_y;  // Forward/backward
////        double x = gamepad1.left_stick_x * 1.1;  // Strafe
////        double rx = gamepad1.right_stick_x;  // Rotation
////
////        // Calculate motor powers
////        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
////        double frontLeftPower = (y + x + rx) / denominator;
////        double backLeftPower = (y - x + rx) / denominator;
////        double frontRightPower = (y - x - rx) / denominator;
////        double backRightPower = (y + x - rx) / denominator;
////        // different drive train
//////        double frontLeftPower = (y + x + rx) * .9;
//////        double frontRightPower = (y - x - rx) * .9;
//////        double backLeftPower = (y - x + rx) * .9;
//////        double backRightPower = (y + x - rx) * .9;
////
////        backLeft.setPower(backLeftPower);
////        backRight.setPower(backRightPower);
////        frontLeft.setPower(frontLeftPower);
////        frontRight.setPower(frontRightPower);
////    }
////
////
////    private void controlSlides() {
////        // SLIDES movement code
////        if (gamepad2.left_stick_y < 0 && leftSlideMotor.getCurrentPosition() < 8100 && rightSlideMotor.getCurrentPosition() > -8100) {
////            rightSlideMotor.setPower(-gamepad2.left_stick_y * -1);
////            leftSlideMotor.setPower(-gamepad2.left_stick_y * 1);
////        } else if (gamepad2.left_stick_y > 0 && leftSlideMotor.getCurrentPosition() > 0 && rightSlideMotor.getCurrentPosition() < 0) {
////            rightSlideMotor.setPower(-gamepad2.left_stick_y * -1);
////            leftSlideMotor.setPower(-gamepad2.left_stick_y * 1);
////        } else {
////            rightSlideMotor.setPower(0);
////            leftSlideMotor.setPower(0);
////        }
////        // Debugging COMMENT LATER ON
////        if (gamepad2.y) {
////            rightSlideMotor.setPower(-gamepad2.left_stick_y * -0.7);
////            leftSlideMotor.setPower(-gamepad2.left_stick_y * 0.7);
////        }
////    }
////    }
//
//// package org.firstinspires.ftc.teamcode;
//
//// import com.qualcomm.robotcore.util.ElapsedTime;
//// import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//// import com.qualcomm.robotcore.util.ElapsedTime;
//// import com.qualcomm.robotcore.hardware.DcMotor;
//// import com.qualcomm.robotcore.hardware.CRServo;
//// import com.qualcomm.robotcore.hardware.Servo;
//
//
//// @TeleOp(name = "ASTeleOp", group = "TeleOp")
//// public class TeleOpFirst extends OpMode {
//
////     // Define motors for driving
////     private DcMotor frontLeft;
////     private DcMotor frontRight;
////     private DcMotor backLeft;
////     private DcMotor backRight;
//
//
////     // motors for slides
////     private DcMotor leftSlideMotor;
////     private DcMotor rightSlideMotor;
//
//
////     // Servos
////     private Servo leftShoulderServo;
////     private Servo rightShoulderServo;
////     private Servo leftArmServo;
////     private Servo rightArmServo;
////     private Servo clawServo;
////     private Servo spinTakeExtendLeft;
////     private Servo spinTakeExtendRight;
////     private Servo spinTakeRotationLeft;
////     private Servo spinTakeRotationRight;
////     private CRServo spinTakeLeft;
////     private CRServo spinTakeRight;
//
//
////     @Override
////     public void init() {
////         // Drivetrain motors
////         frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
////         frontRight = hardwareMap.get(DcMotor.class, "frontRight");
////         backLeft = hardwareMap.get(DcMotor.class, "backLeft");
////         backRight = hardwareMap.get(DcMotor.class, "backRight");
//
//
////         // Reverse left motors for proper directionality
////         frontLeft.setDirection(DcMotor.Direction.REVERSE);
////         backLeft.setDirection(DcMotor.Direction.REVERSE);
//
//
////         // left and right slide motors initialized
////         leftSlideMotor = hardwareMap.get(DcMotor.class, "leftSlideMotor");
////         rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");
////         // Reset encoders for the slide motors
////         leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////         rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////         // Set them to run using encoders after reset
////         leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////         rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//
////         leftArmServo = hardwareMap.get(Servo.class, "elbowLeft");
////         rightArmServo = hardwareMap.get(Servo.class, "elbowRight");
//
//
////         leftShoulderServo = hardwareMap.get(Servo.class, "shoulderLeft");
////         rightShoulderServo = hardwareMap.get(Servo.class, "shoulderRight");
//
//
////         spinTakeLeft = hardwareMap.get(CRServo.class, "spinTakeLeft");
////         spinTakeRight = hardwareMap.get(CRServo.class, "spinTakeRight");
//
//
////         spinTakeExtendLeft = hardwareMap.get(Servo.class, "leftExtend");
////         spinTakeExtendRight = hardwareMap.get(Servo.class, "rightExtend");
//
//
////         spinTakeRotationRight = hardwareMap.get(Servo.class, "wristRight");
////         spinTakeRotationLeft = hardwareMap.get(Servo.class, "wristLeft");
//
////         clawServo = hardwareMap.get(Servo.class, "clawServo");
////     }
//
////     @Override
////     public void start() {
////         leftShoulderServo.setPosition(0);
////         rightShoulderServo.setPosition(1);
////         leftArmServo.setPosition(.9594);
////         rightArmServo.setPosition(.3033);
////     }
//
//
////     @Override
////     public void loop() {
//
//
////         // ---- Drivetrain Control ----
////         controlDrivetrain();
////         // // ------ Manual Controls ------
////         // controlSpinTakeManual();
////         // // ------ Control all claw things -----
////         // controlClaw();
////         // // ----- Control slides -------
////         // controlSlides();
////         // // ----- update values of servos and motors -----
////         // updateAllTelemetry();
//
//
////         // if (gamepad2.y) {
////         //     leftArmServo.setPosition(1);
////         //     rightArmServo.setPosition(.2733);
////         // } else if (gamepad2.a) {
////         //     leftArmServo.setPosition(.2894);
////         //     rightArmServo.setPosition(1);
////         // }
//
////         if (gamepad2.y) {
////             changeServoPositionBy(leftArmServo, .0001);
////             changeServoPositionBy(rightArmServo, -.0001);
////         } else if (gamepad2.a) {
////             changeServoPositionBy(leftArmServo, -.0001);
////             changeServoPositionBy(rightArmServo, .0001);
////         }
//
////         // telemetry.addData("Left elbow: ", leftArmServo.getPosition());
////         // telemetry.addData("Right elbow: ", rightArmServo.getPosition());
//
////         // if (gamepad2.left_bumper) {
////         //     leftShoulderServo.setPosition(0);
////         //     rightShoulderServo.setPosition(1);
////         // } else if (gamepad2.right_bumper) {
////         //     leftShoulderServo.setPosition(.89);
////         //     rightShoulderServo.setPosition(0);
////         // }
//
////         if (gamepad2.left_bumper) {
////             // changeServoPositionBy(clawServo, .0025);
////             clawServo.setPosition(1);
////         } else if (gamepad2.right_bumper) {
////             clawServo.setPosition(.6794);
////         }
////         telemetry.addData("claw : ", clawServo.getPosition());
//
////         if (gamepad2.right_trigger > 0) {
////             spinTakeRotationLeft.setPosition(0);
////             spinTakeRotationRight.setPosition(1);
////         } else if(gamepad2.left_trigger > 0) {
////             spinTakeRotationLeft.setPosition(.9317);
////             spinTakeRotationRight.setPosition(.0667);
////         }
////         telemetry.addData("Left wrist: ", spinTakeRotationLeft.getPosition());
////         telemetry.addData("Right wrist: ", spinTakeRotationRight.getPosition());
//
////         if (gamepad2.dpad_left) {
////             spinTakeExtendLeft.setPosition(1);
////             spinTakeExtendRight.setPosition(0);
////             // changeServoPositionBy(spinTakeExtendLeft, .001);
////             // changeServoPositionBy(spinTakeExtendRight, -.001);
////         } else if (gamepad2.dpad_right) {
////             // changeServoPositionBy(spinTakeExtendLeft, -.001);
////             // changeServoPositionBy(spinTakeExtendRight, .001);
////             spinTakeExtendLeft.setPosition(.3944);
////             spinTakeExtendRight.setPosition(.6028);
////         }
//
////         if (gamepad1.right_bumper) {
////             spinTakeRight.setPower(1);
////             spinTakeLeft.setPower(-1);
////         } else if (gamepad1.left_bumper) {
////             spinTakeRight.setPower(-1);
////             spinTakeLeft.setPower(1);
////         } else {
////             spinTakeRight.setPower(0);
////             spinTakeLeft.setPower(0);
////         }
//
//
////         if (gamepad2.x) {
////             // timer.reset();
////             leftArmServo.setPosition(.2894);
////             rightArmServo.setPosition(1);
////             leftShoulderServo.setPosition(.89);
////             rightShoulderServo.setPosition(0);
////         } else if (gamepad2.b) {
////             leftShoulderServo.setPosition(0);
////             rightShoulderServo.setPosition(1);
////             leftArmServo.setPosition(.9594);
////         rightArmServo.setPosition(.3033);
////         }
//
////         if (gamepad1.y) {
////             spinTakeExtendLeft.setPosition(.8322);
////             spinTakeExtendRight.setPosition(.3028);
////         }
//
//
//
////         telemetry.addData("elbow servo arm: ", leftArmServo.getPosition());
////         telemetry.addData("elbow servo arm: ", rightArmServo.getPosition());
////         telemetry.addData("Left extend: ", spinTakeExtendLeft.getPosition());
////         telemetry.addData("Right servo: ", spinTakeExtendRight.getPosition());
////         telemetry.addData("Left slide: ", leftSlideMotor.getCurrentPosition());
////         telemetry.addData("Right right: ", rightSlideMotor.getCurrentPosition());
////         telemetry.addData("Left wrist: ", spinTakeRotationLeft.getPosition());
////         telemetry.addData("Right wrist: ", spinTakeRotationRight.getPosition());
////         telemetry.addData("Left Shoulder arm: ", leftShoulderServo.getPosition());
////         telemetry.addData("Right Shoulder arm: ", rightShoulderServo.getPosition());
////         telemetry.update();
//
////         controlSlides();
//
////     }
//
//
////     private void changeServoPositionBy(Servo servo, double delta) {
////         servo.setPosition(servo.getPosition() + delta);
////     }
//
//
////     private void updateAllTelemetry() {
////         // telemetry.addData("SpinTake rotation servo left: ", spinTakeRotationLeft.getPosition());
////         // telemetry.addData("SpinTake rotation servo right: ", spinTakeRotationRight.getPosition());
////         // telemetry.addData("Left servo arm: ", leftArmServo.getPosition());
////         // telemetry.addData("Right servo arm: ", rightArmServo.getPosition());
////         // telemetry.addData("Left Shoulder arm: ", leftShoulderServo.getPosition());
////         // telemetry.addData("Right Shoulder arm: ", rightShoulderServo.getPosition());
////         // telemetry.addData("Left slide: ", leftSlideMotor.getCurrentPosition());
////         // telemetry.addData("Right right: ", rightSlideMotor.getCurrentPosition());
////         telemetry.update();
////     }
//
//
////     private void controlDrivetrain() {
////         double y = -gamepad1.left_stick_y;  // Forward/backward
////         double x = gamepad1.left_stick_x * 1.1;  // Strafe
////         double rx = gamepad1.right_stick_x;  // Rotation
//
//
////         // Calculate motor powers
////         double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
////         double frontLeftPower = (y + x + rx) / denominator;
////         double backLeftPower = (y - x + rx) / denominator;
////         double frontRightPower = (y - x - rx) / denominator;
////         double backRightPower = (y + x - rx) / denominator;
////         // different drive train
//// //        double frontLeftPower = (y + x + rx) * .9;
//// //        double frontRightPower = (y - x - rx) * .9;
//// //        double backLeftPower = (y - x + rx) * .9;
//// //        double backRightPower = (y + x - rx) * .9;
//
//
////         // Set motor powers
////         backLeft.setPower(backLeftPower);
////         backRight.setPower(backRightPower);
////         frontLeft.setPower(frontLeftPower);
////         frontRight.setPower(frontRightPower);
////     }
//
//
////     private void controlSlides() {
////         // SLIDES movement code
////         if (gamepad2.left_stick_y < 0 && leftSlideMotor.getCurrentPosition() < 8100 && rightSlideMotor.getCurrentPosition() > -8100) {
////             rightSlideMotor.setPower(-gamepad2.left_stick_y * -1);
////             leftSlideMotor.setPower(-gamepad2.left_stick_y * 1);
////         } else if (gamepad2.left_stick_y > 0 && leftSlideMotor.getCurrentPosition() > 0 && rightSlideMotor.getCurrentPosition() < 0) {
////             rightSlideMotor.setPower(-gamepad2.left_stick_y * -1);
////             leftSlideMotor.setPower(-gamepad2.left_stick_y * 1);
////         } else {
////             rightSlideMotor.setPower(0);
////             leftSlideMotor.setPower(0);
////         }
////         // Debugging COMMENT LATER ON
////         if (gamepad2.y) {
////             rightSlideMotor.setPower(-gamepad2.left_stick_y * -0.7);
////             leftSlideMotor.setPower(-gamepad2.left_stick_y * 0.7);
////         }
////     }
//
//
////     private void controlSpinTakeManual() {
////         // SpinTake code
////         if (gamepad1.right_bumper) {
////             spinTakeRight.setPower(1);
////             spinTakeLeft.setPower(-1);
////         } else if (gamepad1.left_bumper) {
////             spinTakeRight.setPower(-1);
////             spinTakeLeft.setPower(1);
////         } else {
////             spinTakeRight.setPower(0);
////             spinTakeLeft.setPower(0);
////         }
////         // Rotation of SpinTake
////         if (gamepad2.dpad_left) {
////             changeServoPositionBy(spinTakeRotationLeft, .01);
////             changeServoPositionBy(spinTakeRotationRight, -.01);
////         } else if(gamepad2.dpad_right) {
////             changeServoPositionBy(spinTakeRotationLeft, -.01);
////             changeServoPositionBy(spinTakeRotationRight, .01);
////         }
//
//
////         // Extend for spinTake
////         if (gamepad2.right_trigger > 0) {
////             changeServoPositionBy(spinTakeExtendRight, .01);
////             changeServoPositionBy(spinTakeExtendLeft, -.01);
////         } else if (gamepad1.left_trigger > 0) {
////             changeServoPositionBy(spinTakeExtendRight, -.01);
////             changeServoPositionBy(spinTakeExtendLeft, .01);
////         }
////     }
//
//
////     private void controlClaw() {
////         // CLAW servo
////         if(gamepad2.right_bumper){
////             changeServoPositionBy(clawServo, .001);
////         } else if(gamepad2.left_bumper){
////             changeServoPositionBy(clawServo, -.001);
////         }
//
//
////         // ARM
////         if (gamepad2.left_trigger > 0) {
////             changeServoPositionBy(leftArmServo, .01);
////             changeServoPositionBy(leftArmServo, -.01);
////         } else if (gamepad2.right_trigger > 0) {
////             changeServoPositionBy(leftArmServo, -.01);
////             changeServoPositionBy(leftArmServo, .01);
////         }
//
//
////         // ARM shoulder servo thing
////         if (gamepad2.left_bumper) {
////             changeServoPositionBy(leftShoulderServo, .01);
////             changeServoPositionBy(rightShoulderServo, -.01);
////         } else if (gamepad2.right_bumper) {
////             changeServoPositionBy(leftShoulderServo, -.01);
////             changeServoPositionBy(rightShoulderServo, .01);
////         }
////     }
//
//
//// }
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
//    private Servo extenLeft;
//    private Servo extenRight;
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
//        extenLeft = hardwareMap.get(Servo.class, "leftExtend");
//        extenRight = hardwareMap.get(Servo.class, "rightExtend");
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
////        extenLeft.setPosition(1);
////        extenRight.setPosition(0);
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
//            extenLeft.setPosition(1);
//            extenRight.setPosition(0);
//            // changeServoPositionBy(extenLeft, .001);
//            // changeServoPositionBy(extenRight, -.001);
//        } else if (gamepad2.dpad_right) { // extend extend servos
//            // changeServoPositionBy(extenLeft, -.001);
//            // changeServoPositionBy(extenRight, .001);
//            extenLeft.setPosition(.3944);
//            extenRight.setPosition(.6028);
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
//        telemetry.addData("Left extend: ", extenLeft.getPosition());
//        telemetry.addData("Right extend: ", extenRight.getPosition());
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
//            rightSlideMotor.setPower(-gamepad2.left_stick_y * -.8);
//            leftSlideMotor.setPower(-gamepad2.left_stick_y * .8);
//        } else if (gamepad2.left_stick_y > 0 && leftSlideMotor.getCurrentPosition() > 0 && rightSlideMotor.getCurrentPosition() < 0) {
//            rightSlideMotor.setPower(-gamepad2.left_stick_y * -.8);
//            leftSlideMotor.setPower(-gamepad2.left_stick_y * .8);
//        } else {
//            rightSlideMotor.setPower(0);
//            leftSlideMotor.setPower(0);
//        }
//        // Debugging COMMENT LATER ON
//        // rightSlideMotor.setPower(-gamepad2.right_stick_y * -1);
//        // leftSlideMotor.setPower(-gamepad2.right_stick_y * 1);
//    }
//}
