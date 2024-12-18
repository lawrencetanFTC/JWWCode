package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.util.Telemetry;

@TeleOp(name = "ASTeleOp8y8y8", group = "TeleOp")
public class TeleOpFirst2 extends OpMode {
    // Define motors for driving
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Servos and mechanisms defined
    private Servo clawServo;
    private CRServo spinTakeLeft;
    private CRServo spinTakeRight;
    private Servo leftExtendServo;
    private Servo rightExtendServo;
    private DcMotor leftSlideMotor;
    private DcMotor rightSlideMotor;
    private Servo leftArmServo;
    private Servo rightArmServo;
    private Servo leftWristServo;
    private Servo rightWristServo;
    private Servo latchServo;

    private final double CLAW_OPEN_POSITION = 0.52;
    private final double CLAW_CLOSE_POSITION = 0.65;

    private final double ARM_POSITION_BASKET = 0;

    private double LATCH_OPEN_POSITION;

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

        clawServo = hardwareMap.get(Servo.class, "clawServo");

        // left and right slide motors initialized
        leftSlideMotor = hardwareMap.get(DcMotor.class, "leftSlideMotor");
        rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");
        // Reset encoders for the slide motors
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Set them to run using encoders after reset
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Servos and side motors
        spinTakeLeft = hardwareMap.get(CRServo.class, "spinTakeLeft");
        spinTakeRight = hardwareMap.get(CRServo.class, "spinTakeRight");
        // extend arm servos
        leftExtendServo = hardwareMap.get(Servo.class, "leftExtendServo");
        rightExtendServo = hardwareMap.get(Servo.class, "rightExtendServo");
        // turning arm servos
        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        // wrist servos
        leftWristServo = hardwareMap.get(Servo.class, "leftWristServo");
        rightWristServo = hardwareMap.get(Servo.class, "rightWristServo");
        // LATCH servo
        latchServo = hardwareMap.get(Servo.class, "latchServo");

        LATCH_OPEN_POSITION = latchServo.getPosition();
    }

    double lastRuntime = 0;

    @Override
    public void loop() {
        waitUntilIntervalReached(0.05);

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

        // ------- Wrist CODE -------
        // SPIN TAKE servo code
        if (gamepad1.a) {
            spinTakeLeft.setPower(1);
            spinTakeRight.setPower(-1);
        } else if (gamepad1.b) {
            spinTakeLeft.setPower(-1);
            spinTakeRight.setPower(1);
        } else {
            spinTakeLeft.setPower(0);
            spinTakeRight.setPower(0);
        }
        // CLAW servo
        if (gamepad2.right_bumper) {
            clawServo.setPosition(clawServo.getPosition() + 0.005);
        } else if (gamepad2.left_bumper) {
            clawServo.setPosition(clawServo.getPosition() - 0.005);
        }
        // WRIST servo code
        if (gamepad2.dpad_left) {
            leftWristServo.setPosition(leftWristServo.getPosition() - .005);
            rightWristServo.setPosition(rightWristServo.getPosition() + .005);
        } else if (gamepad2.dpad_right) {
            leftWristServo.setPosition(leftWristServo.getPosition() + .005);
            rightWristServo.setPosition(rightWristServo.getPosition() - .005);
        }

        // -------ARM Code------
        // ARM EXTEND Code
        if (gamepad1.right_trigger > 0) {
            // leftExtendServo.setPower(1);
            // rightExtendServo.setPower(-1);
            leftExtendServo.setPosition(leftExtendServo.getPosition() - .005);
            rightExtendServo.setPosition(Math.min(rightExtendServo.getPosition() + .005, Math.abs(1 - leftExtendServo.getPosition())));
        } else if (gamepad1.left_trigger > 0) {
            // leftExtendServo.setPower(-1);
            // rightExtendServo.setPower(1);
            leftExtendServo.setPosition(leftExtendServo.getPosition() + .005);
            rightExtendServo.setPosition(Math.max(rightExtendServo.getPosition() - .005, Math.abs(1 - leftExtendServo.getPosition())));
        }
        // telementry.addData("Left extend servo: ", leftExtendServo.setPosition());
        // telementry.addData("Right extend servo: ", rightExtendServo.setPosition());
        // ARM servo code with constrains
        if (leftSlideMotor.getCurrentPosition() > 0 || rightSlideMotor.getCurrentPosition() > 0) {
            // Extend servos to original position
//            leftExtendServo.setPosition(0); // Example original position (has to be tuned)
//            rightExtendServo.setPosition(0);
            // Arm servos to original position
            leftArmServo.setPosition(ARM_POSITION_BASKET);
            rightArmServo.setPosition(ARM_POSITION_BASKET);
        } else { // if the slides are at 0 position meaning not extended
            if (gamepad2.right_trigger > .0) {
                // Increment arm servo positions
                leftArmServo.setPosition(leftArmServo.getPosition() - 0.005);
                rightArmServo.setPosition(rightArmServo.getPosition() + 0.005);
            } else if (gamepad2.left_trigger > .0) {
                // Decrement arm servo positions
                leftArmServo.setPosition(leftArmServo.getPosition() + 0.005);
                rightArmServo.setPosition(rightArmServo.getPosition() - 0.005);
            }
        }
        // telementry.addData("Left servo: ", leftArmServo.setPosition());
        // telementry.addData("Right servo: ", rightArmServo.setPosition());

        // SLIDES movement code
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
        // Debugging COMMENT LATER ON
        if (gamepad2.y) {
            rightSlideMotor.setPower(-gamepad2.left_stick_y * -0.7);
            leftSlideMotor.setPower(-gamepad2.left_stick_y * 0.7);
        }

        telemetry.addData("Latch Position", latchServo.getPosition());
        if (gamepad1.x) {
            latchServo.setPosition(latchServo.getPosition() + .003);
        } else if (gamepad1.y) {
            latchServo.setPosition(latchServo.getPosition() - .003);
        }


    }

    private void waitUntilIntervalReached(double interval) {
        double rt = this.getRuntime();
        if (lastRuntime != 0) {
            double diff = rt - lastRuntime;
            if (diff < interval) {
                try {
                    Thread.sleep((int) (interval - diff) * 1000);
                } catch (InterruptedException e) {
                    // ignore
                }
            }
        }
        lastRuntime = rt;
    }
}
