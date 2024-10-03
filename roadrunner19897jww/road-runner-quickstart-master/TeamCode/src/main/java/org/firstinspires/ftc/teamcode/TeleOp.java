package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOpFirst")
public class TeleOpFirst extends OpMode {
    // Define motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Servo positions
    // private double extendServoPosition = 0.0;
    // private double armServoPosition = 0.0;
    // private final double SERVO_INCREMENT = 0.05;

//    private Servo clawServo;
//    private Servo holdServo;
//    private Servo extendServo;
//    private Servo armServo;

    @Override
    public void init() {
        // Initialize the motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior (optional but recommended)
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize the servos if needed
        // clawServo = hardwareMap.get(Servo.class, "clawServo");
        // holdServo = hardwareMap.get(Servo.class, "holdServo");
        // extendServo = hardwareMap.get(Servo.class, "extendServo");
        // armServo = hardwareMap.get(Servo.class, "armServo");
    }

    @Override
    public void loop() {
        // Mecanum drive logic
        double y = -gamepad1.left_stick_y; // Forward/backward
        double x = gamepad1.left_stick_x * 1.1; // Strafing, scaled for more precision
        double turn = gamepad1.right_stick_x; // Turning

        // Mecanum drive formula
        double frontLeftPower = .75 * (y + x + turn);
        double frontRightPower = .75 * (y - x - turn);
        double backLeftPower = .75 * (y - x + turn);
        double backRightPower = .75 * (y + x - turn);

        // Set motor powers
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        // Servo control
        // if (gamepad1.a) {
        //     clawServo.setPosition(0.5);
        // } else if (gamepad1.b) {
        //     clawServo.setPosition(0.0);
        // }

        // if (gamepad1.x) {
        //     holdServo.setPosition(0.5);
        // } else if (gamepad1.y) {
        //     holdServo.setPosition(0.0);
        // }

        // // Arm and extension control
        // if (gamepad2.a) {
        //     extendServoPosition += SERVO_INCREMENT;
        //     extendServoPosition = Math.min(extendServoPosition, 1.0);
        // } else if (gamepad2.b) {
        //     extendServoPosition -= SERVO_INCREMENT;
        //     extendServoPosition = Math.max(extendServoPosition, 0.0);
        // }
        // extendServo.setPosition(extendServoPosition);

        // if (gamepad2.x) {
        //     armServoPosition += SERVO_INCREMENT;
        //     armServoPosition = Math.min(armServoPosition, 1.0);
        // } else if (gamepad2.y) {
        //     armServoPosition -= SERVO_INCREMENT;
        //     armServoPosition = Math.max(armServoPosition, 0.0);
        // }
        // armServo.setPosition(armServoPosition);
    }
}