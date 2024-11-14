package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "WorkTeleOp")
public class WorkTeleOp extends OpMode {
    // Define motors for driving
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Define continuous servos and regular servos
    private CRServo spinTake;            // Continuous servo for intake spin
    private DcMotor armMotor;            // Motor for spin take arm up and down

    // Define servos for Gamepad B controls (e.g., claw)
    private Servo baseServo;             // Servo for arm expansion
    private DcMotor leftSideMotor;       // Motor for left-side mechanism
    private DcMotor rightSideMotor;      // Motor for right-side mechanism

    private double targetPosition = 0;           // Target position for the arm motor
    private double leftSideTargetPosition = 0;   // Target position for the left side motor
    private double rightSideTargetPosition = 0;  // Target position for the right side motor

    private final double Kp = 0.05;      // Proportional gain (tune as needed)
    private final double Ki = 0.01;      // Integral gain (tune as needed)
    private final double Kd = 0.02;      // Derivative gain (tune as needed)

    private double previousErrorArm = 0;         // To calculate derivative for arm
    private double previousErrorLeftSide = 0;    // To calculate derivative for left side motor
    private double previousErrorRightSide = 0;   // To calculate derivative for right side motor

    private double integralSumArm = 0;           // To accumulate the integral for arm
    private double integralSumLeftSide = 0;      // To accumulate the integral for left side motor
    private double integralSumRightSide = 0;     // To accumulate the integral for right side motor

    private final double LEFT_SIDE_MAX_POSITION = 1000;  // Adjust these limits as needed
    private final double LEFT_SIDE_MIN_POSITION = 0;
    private final double RIGHT_SIDE_MAX_POSITION = 1000; // Adjust for the correct range
    private final double RIGHT_SIDE_MIN_POSITION = 0;

    @Override
    public void init() {
        // Initialize the motors for driving
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Reverse the motor on the left side for proper directionality
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Initialize servos and motor for mechanisms
        spinTake = hardwareMap.get(CRServo.class, "spinTake");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftSideMotor = hardwareMap.get(DcMotor.class, "leftSideMotor");
        rightSideMotor = hardwareMap.get(DcMotor.class, "rightSideMotor");

        // Set initial target positions to the current positions of the motors
        targetPosition = armMotor.getCurrentPosition();
        leftSideTargetPosition = leftSideMotor.getCurrentPosition();
        rightSideTargetPosition = rightSideMotor.getCurrentPosition();
    }

    @Override
    public void loop() {
        // **Driving Controls for Gamepad A**
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double turn = gamepad1.right_stick_x;

        double frontLeftPower = 0.75 * (-x - y - turn);
        double frontRightPower = (y - x - turn) * 0.90;
        double backLeftPower = (y - x + turn) * 0.85;
        double backRightPower = (y + x - turn) * 0.75;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        // **Arm Motor with PID Control**
        if (gamepad1.left_bumper) {
            targetPosition += 20;  // Adjust upward by an increment
        } else if (gamepad1.right_bumper) {
            targetPosition -= 20;  // Adjust downward by an increment
        }

        pidControl(armMotor, targetPosition, Kp, Ki, Kd, previousErrorArm, integralSumArm);

        // **Left Side Motor with PID Control**
        if (gamepad2.dpad_up) {
            leftSideTargetPosition += 10;
        } else if (gamepad2.dpad_down) {
            leftSideTargetPosition -= 10;
        }

        // Left Side Motor with PID Control and Boundary Limits
//        if (gamepad2.dpad_up && leftSideTargetPosition < LEFT_SIDE_MAX_POSITION) {
//            leftSideTargetPosition += 10;
//        } else if (gamepad2.dpad_down && leftSideTargetPosition > LEFT_SIDE_MIN_POSITION) {
//            leftSideTargetPosition -= 10;
//        }

        pidControl(leftSideMotor, leftSideTargetPosition, Kp, Ki, Kd, previousErrorLeftSide, integralSumLeftSide);

        // **Right Side Motor with PID Control**
        if (gamepad2.dpad_up) {
            rightSideTargetPosition += 10;
        } else if (gamepad2.dpad_down) {
            rightSideTargetPosition -= 10;
        }

        pidControl(rightSideMotor, rightSideTargetPosition, Kp, Ki, Kd, previousErrorRightSide, integralSumRightSide);

        // SpinTake servo with protection to avoid overheating if pressed too long
        if (gamepad1.a) {
            spinTake.setPower(1);
        } else if (gamepad1.b) {
            spinTake.setPower(-1);
        } else {
            spinTake.setPower(0);
        }
    }

    private void pidControl(DcMotor motor, double targetPos, double Kp, double Ki, double Kd, double previousError, double integralSum) {
        double currentPosition = motor.getCurrentPosition();
        double error = targetPos - currentPosition;

        // Proportional term
        double P = error * Kp;

        // Integral term
        integralSum += error;
        double I = integralSum * Ki;

        // Derivative term
        double derivative = error - previousError;
        double D = derivative * Kd;
        previousError = error;

        // Total PID output
        double pidOutput = P + I + D;
        motor.setPower(pidOutput);

        // Reset integral if error is minimal
        if (Math.abs(error) < 10) {
            integralSum = 0;
        }
    }
}
