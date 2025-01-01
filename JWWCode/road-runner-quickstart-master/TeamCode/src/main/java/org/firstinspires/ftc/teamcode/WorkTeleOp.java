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
