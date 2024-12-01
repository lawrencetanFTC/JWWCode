package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ASTeleOp12345", group = "TeleOp")
public class TeleOpFirst2 extends OpMode {
    // Define motors for driving
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Servos and mechanisms
    private Servo clawServo;
    private CRServo spinTake;
    private Servo leftExtendServo;
    private Servo rightExtendServo;
    private DcMotor leftSideMotor;
    private DcMotor rightSideMotor;
    private Servo leftArmServo;
    private Servo rightArmServo;
    private Servo leftWristServo;
    private Servo rightWristServo;

    // PID constants for arm control
    // private final double Kp = 0.5;
    // private final double Ki = 0.02;
    // private final double Kd = 0.03;
    // private double targetPosition = 0;
    // private double integralSum = 0;
    // private double lastError = 0;

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

        // Servos and side motors
        leftSideMotor = hardwareMap.get(DcMotor.class, "leftSideMotor");
        rightSideMotor = hardwareMap.get(DcMotor.class, "rightSideMotor");
        spinTake = hardwareMap.get(CRServo.class, "spinTake");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        leftExtendServo = hardwareMap.get(Servo.class, "leftExtendServo");
        rightExtendServo = hardwareMap.get(Servo.class, "rightExtendServo");
        leftWristServo = hardwareMap.get(Servo.class, "leftWristServo");
        rightWristServo = hardwareMap.get(Servo.class, "rightWristServo");
    }

    @Override
    public void loop() {
        // ---- Drivetrain Control ----
        double y = -gamepad1.left_stick_y; // Forward/backward
        double x = gamepad1.left_stick_x; // Strafing
        double turn = gamepad1.right_stick_x; // Rotation

        // Mecanum drive formulas
        double frontLeftPower = (y + x + turn) * 0.80;
        double frontRightPower = (y - x - turn) * 0.80;
        double backLeftPower = (y - x + turn) * 0.80;
        double backRightPower = (y + x - turn) * 0.80;

        // Apply power
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        if (gamepad2.dpad_up) {
            leftExtendServo.setPosition(1);
            rightExtendServo.setPosition(1);
        }
        else {
            leftExtendServo.setPosition(0);
            rightExtendServo.setPosition(0);
        }

        if (gamepad1.a) {
            spinTake.setPower(1);
        }
        else {
            spinTake.setPower(0);
        }

        // if (gamepad2.left_bumper) {
        // clawServo.setPosition(CLAW_OPEN_POSITION);
        // }
        // else if (gamepad2.right_bumper) {
        // clawServo.setPosition(CLAW_CLOSE_POSITION);
        // } Make sure to uncomment it after both positions are given by builders

        if (gamepad2.a) {
            leftArmServo.setPosition(1);
            rightArmServo.setPosition(1);
        }
        else {
            leftArmServo.setPosition(0);
            rightArmServo.setPosition(0);
        }


        int leftSlidePosition = leftSideMotor.getCurrentPosition();
        int rightSlidePosition = rightSideMotor.getCurrentPosition();

        leftSideMotor.setPower(-gamepad2.left_stick_y * 1); // Reduced speed to 50%
        rightSideMotor.setPower(-gamepad2.left_stick_y * -1);

    }


// if (gamepad1.dpad_up) {
    // targetPosition += NORMAL_INCREMENT; // Increase target position
    // double currentPosition = armMotor.getCurrentPosition();

    // Calculate PID terms
    // double error = targetPosition - currentPosition;
    // integralSum += error; // Accumulate error for integral term
    // double derivative = error - lastError; // Change in error for derivative term

    // Compute motor power using PID
    // double power = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

    // Apply power to both motors
    // armMotor.setPower(power);
    // armMotor2.setPower(power);

    // Update last error
    // lastError = error;

    // else if (gamepad1.dpad_down) {
    // targetPosition -= NORMAL_INCREMENT; // Decrease target position
    // double currentPosition = armMotor.getCurrentPosition();

    // Calculate PID terms
    // double error = targetPosition - currentPosition;
    // integralSum += error; // Accumulate error for integral term
    // double derivative = error - lastError; // Change in error for derivative term

    // Compute motor power using PID
    // double power = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

    // Apply power to both motors
    // armMotor.setPower(power);
    // armMotor2.setPower(power);

    // Update last error
    // lastError = error;

// private void pidControl(double increment) {
    // targetPosition += increment; // Adjust target position

    // double currentPosition1 = armMotor.getCurrentPosition();
    // double currentPosition2 = armMotor2.getCurrentPosition();
    // double averagePosition = (currentPosition1 + currentPosition2) / 2;

    // double error = targetPosition - averagePosition;
    // integralSum += error;
    // double derivative = error - lastError;

    // double power = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

    // armMotor.setPower(power);
    // armMotor2.setPower(-power);

    // lastError = error;
}