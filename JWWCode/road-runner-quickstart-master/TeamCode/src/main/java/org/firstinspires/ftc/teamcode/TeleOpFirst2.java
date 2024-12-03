package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ASTeleOp122345", group = "TeleOp")
public class TeleOpFirst2 extends OpMode {
    // Define motors for driving
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Servos and mechanisms
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

    // Constants for claw servo
//    private final double CLAW_OPEN_POSITION = 0.52;
//    private final double CLAW_CLOSE_POSITION = 0.65;
    boolean extendinit = false;

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

        // clawServo = hardwareMap.get(Servo.class, "clawServo");

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
        // // extend arm servos
        leftExtendServo = hardwareMap.get(Servo.class, "leftExtendServo");
        rightExtendServo = hardwareMap.get(Servo.class, "rightExtendServo");
        // arm turning servos
        // leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        // rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        // wriste turn thing servos :)
        leftWristServo = hardwareMap.get(Servo.class, "leftWristServo");
        rightWristServo = hardwareMap.get(Servo.class, "rightWristServo");
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

        // if (gamepad1.a) {
        //     spinTake.setPower(1);
        // } else if (gamepad1.b) {
        //     spinTake.setPower(-1);
        // } else {
        //     spinTake.setPower(0);
        // }

        // if (gamepad2.left_bumper) {
        //     clawServo.setPosition(CLAW_OPEN_POSITION);
        // } else if (gamepad2.right_bumper) {
        //     clawServo.setPosition(CLAW_CLOSE_POSITION);
        // }

        // SPIN TAKE servo code
        if (gamepad1.a) {
            spinTakeLeft.setPower(1);
            spinTakeRight.setPower(-1);
        } else if (gamepad1.b) {
            spinTakeLeft.setPower(-1);
            spinTakeRight.setPower(1);
        }
        else {
            spinTakeLeft.setPower(0);
            spinTakeRight.setPower(0);
        }

        // ARM servos
        if (gamepad2.left_bumper) {
            leftWristServo.setPosition(.3);
            // rightWristServo.setPosition(-.1);
        } else if (gamepad2.right_bumper) {
            leftWristServo.setPosition(-.3);
            // rightWristServo.setPosition(.1);
        }

        if (gamepad2.left_trigger > 0) {
            // leftArmServo.setPosition(leftArmServo.getPosition() + .1);
            // rightArmServo.setPosition(rightArmServo.getPosition() + -.1);
        } else if (gamepad2.right_trigger > 0) {
            // leftArmServo.setPosition(leftArmServo.getPosition() + -.1);
            // rightArmServo.setPosition(rightArmServo.getPosition() + .1);
        }

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

        if (gamepad2.y) {
            rightSlideMotor.setPower(-gamepad2.left_stick_y * -0.7);
            leftSlideMotor.setPower(-gamepad2.left_stick_y * 0.7);
        }
        if(gamepad2.dpad_up){

            leftExtendServo.setPosition(leftExtendServo.getPosition() - .005);
            rightExtendServo.setPosition(Math.min(rightExtendServo.getPosition() + .005,Math.abs(1 - leftExtendServo.getPosition() )));

        }else if(gamepad2.dpad_down) {
            leftExtendServo.setPosition(leftExtendServo.getPosition() + .005);
            rightExtendServo.setPosition(Math.max(rightExtendServo.getPosition() - .005, Math.abs(1 - leftExtendServo.getPosition())));

        }

        if(gamepad2.dpad_left){
            leftWristServo.setPosition(leftWristServo.getPosition() - .05);
            rightWristServo.setPosition(rightWristServo.getPosition() + .05);
        }else if(gamepad2.dpad_right){
            leftWristServo.setPosition(leftWristServo.getPosition() + .05);
            rightWristServo.setPosition(rightWristServo.getPosition() - .05);
        }

        if(gamepad2.x){
            // clawServo.setPosition(0);
        } else if(gamepad2.a){
            // clawServo.setPosition(.4);
        }
        if(gamepad2.right_bumper){
            //clawServo.setPosition(0);
        } else if(gamepad2.left_bumper){
            //clawServo.setPosition(.4);
        }



    }
}
