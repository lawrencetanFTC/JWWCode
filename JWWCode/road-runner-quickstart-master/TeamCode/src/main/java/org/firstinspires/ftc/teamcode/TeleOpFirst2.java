package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ASTeleOpSecondPID", group = "TeleOp")
public class TeleOpFirst2 extends OpMode {
    // Define motors for driving
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Servos and mechanisms
    private Servo clawServo;
    private CRServo spinTake;
    private DcMotorEx armMotor; // Using DcMotorEx for advanced control
    private CRServo extendServo;
    private DcMotor leftSideMotor;
    private DcMotor rightSideMotor;

    // PID constants and variables for arm control
    private final double Kp = 0.5; // Proportional gain
    private final double Ki = 0; // Integral gain
    private final double Kd = 0; // Derivative gain
    private double targetPosition = 0;
    private double integralSum = 0;
    private double lastError = 0;

    // Arm rotation limits
    private final int ARM_LIMIT_UP = 60; // 60 degrees
    private final int ARM_LIMIT_DOWN = 45; // 45 degrees
    private final int TICKS_PER_DEGREE = 4; // 4 ticks per degree for 117 RPM goBilda motor
    private final int MAX_ARM_TICKS = ARM_LIMIT_UP * TICKS_PER_DEGREE;
    private final int MIN_ARM_TICKS = ARM_LIMIT_DOWN * TICKS_PER_DEGREE;

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
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Servos and side motors
        spinTake = hardwareMap.get(CRServo.class, "spinTake");
        extendServo = hardwareMap.get(CRServo.class, "extendServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        leftSideMotor = hardwareMap.get(DcMotor.class, "leftSideMotor");
        rightSideMotor = hardwareMap.get(DcMotor.class, "rightSideMotor");
    }

    @Override
    public void loop() {
        // ---- Drivetrain Control ----
        double y = -gamepad1.left_stick_y; // Forward/backward
        double x = gamepad1.left_stick_x; // Strafing
        double turn = gamepad1.right_stick_x; // Rotation

        // Use original formulas for mecanum drive
        double frontLeftPower = (y + x + turn) * .79;
        double frontRightPower = (y - x - turn) * .79;
        double backLeftPower = (y - x + turn) * .90;
        double backRightPower = (y + x - turn) * .90;

        // Apply power directly
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        // ---- Arm Control with PID and Rotation Limits ----
        // Control for arm movement

        while (gamepad2.right_stick_y > 0) {
            targetPosition += 0.04;
        }

        // Apply PID control
        int currentPosition = armMotor.getCurrentPosition();
        double error = targetPosition - currentPosition;
        integralSum += error;
        double derivative = error - lastError;

        double powerOutput = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        if (powerOutput <= 0.5) {
            armMotor.setPower(powerOutput);
        } else {
            armMotor.setPower(0);
        }
        lastError = error;

        // Update motor position target


        // Telemetry for debugging
        telemetry.addData("Arm Target", targetPosition);
        telemetry.addData("Arm Position", currentPosition);
        telemetry.addData("Arm Power", powerOutput);

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

        // ---- Slide Motor Limits ----
        int leftSlidePosition = leftSideMotor.getCurrentPosition();
        int rightSlidePosition = rightSideMotor.getCurrentPosition();

        if (gamepad2.left_stick_y > 0.1) {
            leftSideMotor.setPower(0.8);
        } else if (gamepad2.left_stick_y > 0.1) {
            leftSideMotor.setPower(-0.8);
        } else {
            leftSideMotor.setPower(0);
        }

        if (gamepad2.left_stick_y > 0.1 ) {
            rightSideMotor.setPower(0.8);
        } else if (gamepad2.left_stick_y > 0.1) {
            rightSideMotor.setPower(-0.8);
        } else {
            rightSideMotor.setPower(0);
        }
        telemetry.update();
    }
}

