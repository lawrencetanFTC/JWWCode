package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ASTeleOp1234567", group = "TeleOp")
public class TeleOpFirst2 extends OpMode {
    // Define motors for driving
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Servos and mechanisms
    private Servo clawServo;
    private CRServo spinTake;

    // armMotors
    private DcMotor armMotor;
    private DcMotor armMotor2;

    private CRServo extendServo;
    private DcMotor leftSlideMotor;
    private DcMotor rightSlideMotor;

    // PID constants and variables for arm control
    private final double Kp = 0.5; // Proportional gain
    private final double Ki = 0; // Integral gain
    private final double Kd = 0; // Derivative gain
    private double targetPosition = 0;
    private double integralSum = 0;
    private double lastError = 0;

    private final int SLIDE_MIN_LIMIT = 0; // Minimum slide position (fully retracted)
    private final int SLIDE_MAX_LIMIT = 500; // Maximum slide position (fully extended)

    // Constants for claw servo
    private final double CLAW_OPEN_POSITION = 0.52;
    private final double CLAW_CLOSE_POSITION = 0.65;

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

        // Arm motor setup using DcMotorEx for better position control
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor2 = hardwareMap.get(DcMotor.class, "armMotor2");
        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor2.setDirection(DcMotor.Direction.REVERSE);

        // Servos and side motors
        spinTake = hardwareMap.get(CRServo.class, "spinTake");
        extendServo = hardwareMap.get(CRServo.class, "extendServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        leftSlideMotor = hardwareMap.get(DcMotor.class, "leftSlideMotor");
        rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");

        // Reset encoders for the slide motors
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set them to run using encoders after reset
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void loop() {
        // ---- Drivetrain Control ----
        double y = -gamepad1.left_stick_y; // Forward/backward
        double x = gamepad1.left_stick_x; // Strafing
        double turn = gamepad1.right_stick_x; // Rotation

        // Use original formulas for mecanum drive
        double frontLeftPower = (y + x + turn) * .85;
        double frontRightPower = (y - x - turn) * .85;
        double backLeftPower = (y - x + turn) * .85;
        double backRightPower = (y + x - turn) * .85;

        // Apply power directly
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        // ---- Servo Control ----
        if (gamepad2.dpad_up) {
            extendServo.setPower(1);
        } else if (gamepad2.dpad_down) {
            extendServo.setPower(-1);
        } else {
            extendServo.setPower(0);
        }

        if (gamepad1.a) {
            spinTake.setPower(1);
        } else if (gamepad1.b) {
            spinTake.setPower(-1);
        } else {
            spinTake.setPower(0);
        }

        if (gamepad2.left_bumper) {
            clawServo.setPosition(CLAW_OPEN_POSITION);
        } else if (gamepad2.right_bumper) {
            clawServo.setPosition(CLAW_CLOSE_POSITION);
        }

//        if (gamepad2.a) {
//            armMotor.setPower(-1);
//            armMotor2.setPower(1);
//        } else if (gamepad2.y) {
//            armMotor.setPower(1);
//            armMotor2.setPower(-1);
//        }
//        else {
//            armMotor.setPower(0);
//            armMotor2.setPower(0);
//        }

        double NORMAL_INCREMENT = .001;

        if (gamepad1.dpad_up) {
            targetPosition += NORMAL_INCREMENT;
        } else if (gamepad1.dpad_down) {
            targetPosition -= NORMAL_INCREMENT; // Adjust target position
        }
        pidControl();
        // armMotor.setConstraints(new TrajectoryVelocityConstraint(9.65), new TrajectoryAcceleration(9.65));

        if (gamepad2.left_bumper) {
            // armMotor.setTargetPosition(5);
            // armMotor2.setTargetPosition(5);
            // armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(.5);
            armMotor2.setPower(.5);
        } else if (gamepad2.right_bumper) {
            // armMotor.setTargetPosition(-5);
            // armMotor2.setTargetPosition(-5);
            // armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(-.5);
            armMotor2.setPower(-.5);
        } else {
            armMotor2.setPower(0);
            armMotor.setPower(0);
        }

        // slides with limits
        // Get user input for slide power
        // double slideInputPower = -gamepad2.left_stick_y * 0.85;

        // Calculate power for both motors with limits
        // double leftPower = calculateMotorPower(leftSlideMotor, slideInputPower, SLIDE_MIN_LIMIT, SLIDE_MAX_LIMIT);
        // double rightPower = calculateMotorPower(rightSlideMotor, slideInputPower, SLIDE_MIN_LIMIT, SLIDE_MAX_LIMIT);

        // leftSlideMotor.setPower(leftPower);
        // rightSlideMotor.setPower(rightPower);

        // rightSlideMotor.setConstraints(new TrajectoryVelocityConstraint(9.65), new TrajectoryAcceleration(9.65));
        // leftSllideMotor.setConstraints(new TrajectoryVelocityConstraint(9.65), new TrajectoryAcceleration(9.65));

        if (gamepad2.left_stick_y < 0 && leftSlideMotor.getCurrentPosition() < 8100 && rightSlideMotor.getCurrentPosition() > -8100) {
            rightSlideMotor.setPower(-gamepad2.left_stick_y * -0.7);
            leftSlideMotor.setPower(-gamepad2.left_stick_y * 0.7);
        } else if (gamepad2.left_stick_y > 0 && leftSlideMotor.getCurrentPosition() > 0 && rightSlideMotor.getCurrentPosition() < 0) {
            rightSlideMotor.setPower(-gamepad2.left_stick_y * -0.7);
            leftSlideMotor.setPower(-gamepad2.left_stick_y * 0.7);
        } else {
            rightSlideMotor.setPower(0);
            leftSlideMotor.setPower(0);
        }

        if (gamepad2.y) {
            rightSlideMotor.setPower(-gamepad2.left_stick_y * -0.7);
            leftSlideMotor.setPower(-gamepad2.left_stick_y * 0.7);
        }



        telemetry.addData("arm 1: ", armMotor.getCurrentPosition());
        telemetry.addData("arm 2", armMotor2.getCurrentPosition());



        telemetry.addData("Arm Motor 1 Position: ", armMotor.getCurrentPosition());
        telemetry.addData("Arm Motor 2 Position: ", armMotor2.getCurrentPosition());
        telemetry.addData("PID Target Position: ", targetPosition);
        telemetry.addData("Error: ", targetPosition - armMotor.getCurrentPosition());



    }

    private void pidControl() {
        int currentPosition1 = armMotor.getCurrentPosition();
        int currentPosition2 = armMotor2.getCurrentPosition();
        double averagePosition = ((double) currentPosition1 + currentPosition2) / 2;
        double feedforward = 0.2;

        double error = targetPosition - averagePosition;
        integralSum += error;
        double derivative = error - lastError;

        double power = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
        double scale = 1.0 / 15;  // scale power approx in range of -1 to 1
        power = power * scale;
        power = Math.signum(power) * Math.min(0.7, Math.abs(power));  // max power is 0.7 or -0.7
        if(power == 0){
            power = feedforward;
        }

        armMotor.setPower(power);
        armMotor2.setPower(power);

        lastError = error;
    }

    private double calculateMotorPower(DcMotor motor, double inputPower, int minLimit, int maxLimit) {
        int currentPosition = motor.getCurrentPosition();

        if ((inputPower > 0 && currentPosition >= maxLimit) ||
                (inputPower < 0 && currentPosition <= minLimit)) {
            return 0;
        }
        return inputPower;
    }
}

