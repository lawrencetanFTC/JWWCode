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
    private CRServo rackAndPinionServo;  // Continuous servo for rack and pinion extension
    private DcMotor armMotor;            // Motor for spin take arm up and down

    // Define servos for Gamepad B controls (e.g., claw)
    private Servo clawServo;             // Servo for claw control
    private DcMotor leftSideMotor;       // Motor for left-side mechanism
    private DcMotor rightSideMotor;      // Motor for right-side mechanism

    // Claw positions
    private final double CLAW_OPEN_POSITION = 0.8;
    private final double CLAW_CLOSE_POSITION = 0.2;

    @Override
    public void init() {
        // Initialize the motors for driving
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Reverse the motors on the left side for proper directionality
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Initialize servos and motor for mechanisms
        spinTake = hardwareMap.get(CRServo.class, "spinTake");                  // Spin intake
        rackAndPinionServo = hardwareMap.get(CRServo.class, "rackAndPinion");   // Rack and pinion extension
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");                  // Arm up/down movement

        // Initialize claw and side mechanism controls (for Gamepad B)
        clawServo = hardwareMap.get(Servo.class, "clawServo");                  // Claw servo
        leftSideMotor = hardwareMap.get(DcMotor.class, "leftSideMotor");        // Left-side mechanism
        rightSideMotor = hardwareMap.get(DcMotor.class, "rightSideMotor");      // Right-side mechanism
    }

    @Override
    public void loop() {
        // Mecanum drive logic for strafing, moving forward/backward, and rotating (Gamepad A)
        double y = -gamepad1.left_stick_y; // Forward/backward
        double x = gamepad1.left_stick_x * 1.1; // Strafing, scaled for more precision
        double turn = gamepad1.right_stick_x; // Turning/rotation

        // Calculate power for each wheel (mecanum wheel configuration)
        double frontLeftPower = .75 * (x - y - turn);
        double frontRightPower = (y - x - turn) * .75;
        double backLeftPower = (y - x + turn) * .75;
        double backRightPower = (y + x - turn) * .75;

        // Set power to the motors
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        // **Gamepad A Controls (Already Implemented)**
        // Arm Movement (Spin take arm up/down with bumpers)
        if (gamepad1.left_bumper) {
            armMotor.setPower(0.5); // Spin take arm up
        } else if (gamepad1.right_bumper) {
            armMotor.setPower(-0.5); // Spin take arm down
        } else {
            armMotor.setPower(0); // Stop arm movement when no button is pressed
        }

        // Spin take control (Continuous servo for intake)
        if (gamepad1.a) {
            spinTake.setPower(1.0); // Spin take forward
        } else if (gamepad1.b) {
            spinTake.setPower(-1.0); // Spin take reverse
        } else {
            spinTake.setPower(0); // Stop spin take when no button is pressed
        }

        // Rack and Pinion control (Continuous servo for extending mechanism)
        if (gamepad1.x) {
            rackAndPinionServo.setPower(1.0); // Extend rack and pinion forward
        } else if (gamepad1.y) {
            rackAndPinionServo.setPower(-1.0); // Retract rack and pinion
        } else {
            rackAndPinionServo.setPower(0); // Stop movement when no button is pressed

            // **Gamepad B Controls**

            // Claw control (Open/Close with bumpers)
            if (gamepad2.left_bumper) {
                clawServo.setPosition(CLAW_OPEN_POSITION); // Open claw
            } else if (gamepad2.right_bumper) {
                clawServo.setPosition(CLAW_CLOSE_POSITION); // Close claw
            }

            // Left side control (Left stick Y-axis)
            double leftSidePower = -gamepad2.left_stick_y; // Y-axis of left stick for control
            leftSideMotor.setPower(leftSidePower);

            // Right side control (Right stick Y-axis)
            double rightSidePower = -gamepad2.right_stick_y; // Y-axis of right stick for control
            rightSideMotor.setPower(rightSidePower);
        }
    }
}
