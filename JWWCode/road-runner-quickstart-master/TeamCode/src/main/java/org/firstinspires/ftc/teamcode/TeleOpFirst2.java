//////package org.firstinspires.ftc.teamcode;
//////
//////import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//////import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//////import com.qualcomm.robotcore.hardware.DcMotor;
//////import com.qualcomm.robotcore.hardware.CRServo;
//////import com.qualcomm.robotcore.hardware.Servo;
//////
//////@TeleOp(name = "ASTeleOp", group = "TeleOp")
//////public class TeleOpFirst extends OpMode {
//////    // Define motors for driving
//////    private DcMotor frontLeft;
//////    private DcMotor frontRight;
//////    private DcMotor backLeft;
//////    private DcMotor backRight;
//////
//////    // Servos and mechanisms defined
//////    private Servo clawServo;
//////    private CRServo spinTakeLeft;
//////    private CRServo spinTakeRight;
//////    private Servo leftExtendServo;
//////    private Servo rightExtendServo;
//////    private DcMotor leftSlideMotor;
//////    private DcMotor rightSlideMotor;
//////    private Servo leftArmServo;
//////    private Servo rightArmServo;
//////    private Servo leftWristServo;
//////    private Servo rightWristServo;
//////    private Servo latchServo;
//////
//////    private final double CLAW_OPEN_POSITION = 0.52;
//////    private final double CLAW_CLOSE_POSITION = 0.65;
//////
//////    private final double ARM_ORIGINAL_POSITION = 0;
//////    private final double ARM_POSITION_BASKET = 0;
//////    private final double ARM_POSITION_SPECIMENT = 0;
//////    private final double ARM_POSITION_HANG = 0;
//////    private final double ARM_POSITION_SPECIMENT_OBSERV = 0;
//////
//////    private final double ARM_EXTEND_RETRACTED = 0;
//////    private final double ARM_EXTEND_EXTENDED = 0;
//////
//////    private double LATCH_OPEN_POSITION;
//////
//////    @Override
//////    public void init() {
//////        // Drivetrain motors
//////        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
//////        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//////        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
//////        backRight = hardwareMap.get(DcMotor.class, "backRight");
//////
//////        // Reverse left motors for proper directionality
//////        frontLeft.setDirection(DcMotor.Direction.REVERSE);
//////        backLeft.setDirection(DcMotor.Direction.REVERSE);
//////
//////        clawServo = hardwareMap.get(Servo.class, "clawServo");
//////
//////        // left and right slide motors initialized
//////        leftSlideMotor = hardwareMap.get(DcMotor.class, "leftSlideMotor");
//////        rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");
//////        // Reset encoders for the slide motors
//////        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//////        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//////        // Set them to run using encoders after reset
//////        leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//////        rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//////
//////        // Servos and side motors
//////        spinTakeLeft = hardwareMap.get(CRServo.class, "spinTakeLeft");
//////        spinTakeRight = hardwareMap.get(CRServo.class, "spinTakeRight");
//////        // extend arm servos
//////        leftExtendServo = hardwareMap.get(Servo.class, "leftExtendServo");
//////        rightExtendServo = hardwareMap.get(Servo.class, "rightExtendServo");
//////        // turning arm servos
//////        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
//////        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
//////        // wrist servos
//////        leftWristServo = hardwareMap.get(Servo.class, "leftWristServo");
//////        rightWristServo = hardwareMap.get(Servo.class, "rightWristServo");
//////        // LATCH servo
//////        latchServo = hardwareMap.get(Servo.class, "latchServo");
//////
//////        LATCH_OPEN_POSITION = latchServo.getPosition();
//////    }
//////
//////    @Override
//////    public void loop() {
//////        // ---- Drivetrain Control ----
//////        double y = -gamepad1.left_stick_y; // Forward/backward
//////        double x = gamepad1.left_stick_x; // Strafing
//////        double turn = gamepad1.right_stick_x; // Rotation
//////        // Use original formulas for mecanum drive
//////        double frontLeftPower = (y + x + turn) * .85;
//////        double frontRightPower = (y - x - turn) * .85;
//////        double backLeftPower = (y - x + turn) * .85;
//////        double backRightPower = (y + x - turn) * .85;
//////        // Apply power directly
//////        frontLeft.setPower(frontLeftPower);
//////        frontRight.setPower(frontRightPower);
//////        backLeft.setPower(backLeftPower);
//////        backRight.setPower(backRightPower);
//////
//////        // ------- Wrist CODE -------
//////        // SPIN TAKE servo code
//////        if (gamepad1.a) {
//////            spinTakeLeft.setPower(1);
//////            spinTakeRight.setPower(-1);
//////        } else if (gamepad1.b) {
//////            spinTakeLeft.setPower(-1);
//////            spinTakeRight.setPower(1);
//////        } else {
//////            spinTakeLeft.setPower(0);
//////            spinTakeRight.setPower(0);
//////        }
//////        // CLAW servo
//////        if(gamepad2.right_bumper){
//////            clawServo.setPosition(0);
//////        } else if(gamepad2.left_bumper){
//////            clawServo.setPosition(.4);
//////        }
//////        // WRIST servo code
//////        if(gamepad2.dpad_left){
//////            leftWristServo.setPosition(leftWristServo.getPosition() - .005);
//////            rightWristServo.setPosition(rightWristServo.getPosition() + .005);
//////        }else if(gamepad2.dpad_right){
//////            leftWristServo.setPosition(leftWristServo.getPosition() + .005);
//////            rightWristServo.setPosition(rightWristServo.getPosition() - .005);
//////        }
//////
//////        // -------ARM Code------
//////        // ARM EXTEND Code
//////        if(gamepad2.dpad_up){
//////            leftExtendServo.setPosition(leftExtendServo.getPosition() - .005);
//////            rightExtendServo.setPosition(Math.min(rightExtendServo.getPosition() + .005,Math.abs(1 - leftExtendServo.getPosition() )));
//////        }else if(gamepad2.dpad_down) {
//////            leftExtendServo.setPosition(leftExtendServo.getPosition() + .005);
//////            rightExtendServo.setPosition(Math.max(rightExtendServo.getPosition() - .005, Math.abs(1 - leftExtendServo.getPosition())));
//////        }
//////        // ARM servo code with constrains
//////        if (leftSlideMotor.getCurrentPosition() > 0 || rightSlideMotor.getCurrentPosition() > 0) {
//////            // Extend servos to original position
////////            leftExtendServo.setPosition(0); // Example original position (has to be tuned)
////////            rightExtendServo.setPosition(0);
//////            // Arm servos to original position
//////            leftArmServo.setPosition(ARM_POSITION_BASKET);
//////            rightArmServo.setPosition(ARM_POSITION_BASKET);
//////        } else { // if the slides are at 0 position meaning not extended
//////            if (gamepad2.right_trigger > .0) {
//////                // Increment arm servo positions
//////                leftArmServo.setPosition(leftArmServo.getPosition() - 0.02);
//////                rightArmServo.setPosition(rightArmServo.getPosition() + 0.02);
//////            } else if (gamepad2.left_trigger > .0) {
//////                // Decrement arm servo positions
//////                leftArmServo.setPosition(leftArmServo.getPosition() + 0.02);
//////                rightArmServo.setPosition(rightArmServo.getPosition() - 0.02);
//////            }
//////        }
//////
//////        // SLIDES movement code
//////        if (gamepad2.left_stick_y < 0 && leftSlideMotor.getCurrentPosition() < 8100 && rightSlideMotor.getCurrentPosition() > -8100) {
//////            rightSlideMotor.setPower(-gamepad2.left_stick_y * -0.7);
//////            leftSlideMotor.setPower(-gamepad2.left_stick_y * 0.7);
//////        } else if (gamepad2.left_stick_y > 0 && leftSlideMotor.getCurrentPosition() > 0 && rightSlideMotor.getCurrentPosition() < 0) {
//////            rightSlideMotor.setPower(-gamepad2.left_stick_y * -0.7);
//////            leftSlideMotor.setPower(-gamepad2.left_stick_y * 0.7);
//////        } else {
//////            rightSlideMotor.setPower(0);
//////            leftSlideMotor.setPower(0);
//////        }
//////        // Debugging COMMENT LATER ON
//////        if (gamepad2.y) {
//////            rightSlideMotor.setPower(-gamepad2.left_stick_y * -0.7);
//////            leftSlideMotor.setPower(-gamepad2.left_stick_y * 0.7);
//////        }
//////
//////        if (gamepad1.x) {
//////            latchServo.setPosition(LATCH_OPEN_POSITION + .0001);
//////        } else if(gamepad1.y) {
//////            latchServo.setPosition(LATCH_OPEN_POSITION);
//////        }
//////
//////    }
//////}
////
////package org.firstinspires.ftc.teamcode;
////
////import com.qualcomm.robotcore.eventloop.opmode.OpMode;
////import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
////import com.qualcomm.robotcore.hardware.DcMotor;
////import com.qualcomm.robotcore.hardware.CRServo;
////import com.qualcomm.robotcore.hardware.Servo;
////
////@TeleOp(name = "ASTeleOp", group = "TeleOp")
////public class WorkTeleOp extends OpMode {
////    // Define motors for driving
////    private DcMotor frontLeft;
////    private DcMotor frontRight;
////    private DcMotor backLeft;
////    private DcMotor backRight;
////
////    // Servos and mechanisms defined
////    private Servo clawServo;
////    private CRServo spinTakeLeft;
////    private CRServo spinTakeRight;
////    private Servo leftExtendServo;
////    private Servo rightExtendServo;
////    private DcMotor leftSlideMotor;
////    private DcMotor rightSlideMotor;
////    private Servo leftArmServo;
////    private Servo rightArmServo;
////    private Servo leftWristServo;
////    private Servo rightWristServo;
////    private Servo latchServo;
////
////    private double LATCH_OPEN_POSITION;
////    private double ARM_START_POSITION_LEFT;
////    private double ARM_START_POSITION_RIGHT;
////
////    @Override
////    public void init() {
////        // Drivetrain motors
////        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
////        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
////        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
////        backRight = hardwareMap.get(DcMotor.class, "backRight");
////
////        // Reverse left motors for proper directionality
////        frontLeft.setDirection(DcMotor.Direction.REVERSE);
////        backLeft.setDirection(DcMotor.Direction.REVERSE);
////
////        clawServo = hardwareMap.get(Servo.class, "clawServo");
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
////        // Servos and side motors
////        spinTakeLeft = hardwareMap.get(CRServo.class, "spinTakeLeft");
////        spinTakeRight = hardwareMap.get(CRServo.class, "spinTakeRight");
////        // extend arm servos
////        leftExtendServo = hardwareMap.get(Servo.class, "leftExtendServo");
////        rightExtendServo = hardwareMap.get(Servo.class, "rightExtendServo");
////        // turning arm servos
////        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
////        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
////        // wrist servos
////        leftWristServo = hardwareMap.get(Servo.class, "leftWristServo");
////        rightWristServo = hardwareMap.get(Servo.class, "rightWristServo");
////        // LATCH servo
////        latchServo = hardwareMap.get(Servo.class, "latchServo");
////
////        LATCH_OPEN_POSITION = latchServo.getPosition();
////        ARM_START_POSITION_LEFT = leftArmServo.getPosition();
////        ARM_START_POSITION_RIGHT = leftArmServo.getPosition();
////    }
////
////    @Override
////    public void loop() {
////        // ---- Drivetrain Control ----
////        double y = -gamepad1.left_stick_y; // Forward/backward
////        double x = gamepad1.left_stick_x; // Strafing
////        double turn = gamepad1.right_stick_x; // Rotation
////        // Use original formulas for mecanum drive
////        double frontLeftPower = (y + x + turn) * .85;
////        double frontRightPower = (y - x - turn) * .85;
////        double backLeftPower = (y - x + turn) * .85;
////        double backRightPower = (y + x - turn) * .85;
////        // Apply power directly
////        frontLeft.setPower(frontLeftPower);
////        frontRight.setPower(frontRightPower);
////        backLeft.setPower(backLeftPower);
////        backRight.setPower(backRightPower);
////
////        // ------- Wrist CODE -------
////        // SPIN TAKE servo code
////        if (gamepad1.right_trigger > 0) {
////            spinTakeLeft.setPower(1);
////            spinTakeRight.setPower(-1);
////        } else if (gamepad1.left_trigger > 0) {
////            spinTakeLeft.setPower(-1);
////            spinTakeRight.setPower(1);
////        } else {
////            spinTakeLeft.setPower(0);
////            spinTakeRight.setPower(0);
////        }
////        // CLAW servo
////        if(gamepad2.right_bumper){
////            clawServo.setPosition(clawServo.getPosition() + 0.005);
////        } else if(gamepad2.left_bumper){
////            clawServo.setPosition(clawServo.getPosition() - 0.005);
////        }
////        // WRIST servo code
////        if(gamepad2.dpad_left){
////            leftWristServo.setPosition(leftWristServo.getPosition() - .005);
////            rightWristServo.setPosition(rightWristServo.getPosition() + .005);
////        }else if(gamepad2.dpad_right){
////            leftWristServo.setPosition(leftWristServo.getPosition() + .005);
////            rightWristServo.setPosition(rightWristServo.getPosition() - .005);
////        }
////
////        // -------ARM Code------
////        // ARM EXTEND Code
////        if(gamepad1.right_bumper){
////            // leftExtendServo.setPower(1);
////            // rightExtendServo.setPower(-1);
////            leftExtendServo.setPosition(leftExtendServo.getPosition() - .005);
////            rightExtendServo.setPosition(Math.min(rightExtendServo.getPosition() + .005,Math.abs(1 - leftExtendServo.getPosition() )));
////        } else if(gamepad1.left_bumper) {
////            // leftExtendServo.setPower(-1);
////            // rightExtendServo.setPower(1);
////            leftExtendServo.setPosition(leftExtendServo.getPosition() + .005);
////            rightExtendServo.setPosition(Math.max(rightExtendServo.getPosition() - .005, Math.abs(1 - leftExtendServo.getPosition())));
////        }
////        telemetry.addData("Left extend servo: ", leftExtendServo.getPosition());
////        telemetry.addData("Right extend servo: ", rightExtendServo.getPosition());
////        // ARM servo code with constrains
////        if (leftSlideMotor.getCurrentPosition() > 20 || rightSlideMotor.getCurrentPosition() > 20) {
////            leftArmServo.setPosition(ARM_START_POSITION_LEFT);
////            rightArmServo.setPosition(ARM_START_POSITION_RIGHT);
////        } else { // if the slides are at 0 position meaning not extended
////            if (gamepad2.right_trigger > 0) {
////                // Increment arm servo positions
////                leftArmServo.setPosition(leftArmServo.getPosition() - 0.005);
////                rightArmServo.setPosition(rightArmServo.getPosition() + 0.005);
////            } else if (gamepad2.left_trigger > 0) {
////                // Decrement arm servo positions
////                leftArmServo.setPosition(leftArmServo.getPosition() + 0.005);
////                rightArmServo.setPosition(rightArmServo.getPosition() - 0.005);
////            }
////        }
////        telemetry.addData("Left servo arm: ", leftArmServo.getPosition());
////        telemetry.addData("Right servo arm: ", rightArmServo.getPosition());
////
////        // SLIDES movement code
////        if (gamepad2.left_stick_y < 0 && leftSlideMotor.getCurrentPosition() < 8100 && rightSlideMotor.getCurrentPosition() > -8100) {
////            rightSlideMotor.setPower(-gamepad2.left_stick_y * -0.7);
////            leftSlideMotor.setPower(-gamepad2.left_stick_y * 0.7);
////        } else if (gamepad2.left_stick_y > 0 && leftSlideMotor.getCurrentPosition() > 0 && rightSlideMotor.getCurrentPosition() < 0) {
////            rightSlideMotor.setPower(-gamepad2.left_stick_y * -0.7);
////            leftSlideMotor.setPower(-gamepad2.left_stick_y * 0.7);
////        } else {
////            rightSlideMotor.setPower(0);
////            leftSlideMotor.setPower(0);
////        }
////        // Debugging COMMENT LATER ON
////        if (gamepad2.y) {
////            rightSlideMotor.setPower(-gamepad2.left_stick_y * -0.7);
////            leftSlideMotor.setPower(-gamepad2.left_stick_y * 0.7);
////        }
////
////        telemetry.addData("Latch Position", latchServo.getPosition());
////        if (gamepad1.x) {
////            latchServo.setPosition(latchServo.getPosition() + .003);
////        } else if(gamepad1.y) {
////            latchServo.setPosition(latchServo.getPosition() - .003);
////        }
////
////    }
////}
////
//////package org.firstinspires.ftc.teamcode;
//////
//////import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//////import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//////import com.qualcomm.robotcore.hardware.DcMotor;
//////import com.qualcomm.robotcore.hardware.CRServo;
//////import com.qualcomm.robotcore.hardware.Servo;
//////
//////@TeleOp(name = "ASTeleOp", group = "TeleOp")
//////public class TeleOpFirst extends OpMode {
//////    // Define motors for driving
//////    private DcMotor frontLeft;
//////    private DcMotor frontRight;
//////    private DcMotor backLeft;
//////    private DcMotor backRight;
//////
//////    // Servos and mechanisms defined
//////    private Servo clawServo;
//////    private CRServo spinTakeLeft;
//////    private CRServo spinTakeRight;
//////    private Servo leftExtendServo;
//////    private Servo rightExtendServo;
//////    private DcMotor leftSlideMotor;
//////    private DcMotor rightSlideMotor;
//////    private Servo leftArmServo;
//////    private Servo rightArmServo;
//////    private Servo leftWristServo;
//////    private Servo rightWristServo;
//////    private Servo latchServo;
//////
//////    private double LATCH_OPEN_POSITION;
//////    private double ARM_START_POSITION_LEFT;
//////    private double ARM_START_POSITION_RIGHT;
//////
//////    @Override
//////    public void init() {
//////        // Drivetrain motors
//////        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
//////        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//////        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
//////        backRight = hardwareMap.get(DcMotor.class, "backRight");
//////
//////        // Reverse left motors for proper directionality
//////        frontLeft.setDirection(DcMotor.Direction.REVERSE);
//////        backLeft.setDirection(DcMotor.Direction.REVERSE);
//////
////////        clawServo = hardwareMap.get(Servo.class, "clawServo");
//////
//////        // left and right slide motors initialized
//////        leftSlideMotor = hardwareMap.get(DcMotor.class, "leftSlideMotor");
//////        rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");
//////        // Reset encoders for the slide motors
//////        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//////        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//////        // Set them to run using encoders after reset
//////        leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//////        rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//////
////////        // Servos and side motors
////////        spinTakeLeft = hardwareMap.get(CRServo.class, "spinTakeLeft");
////////        spinTakeRight = hardwareMap.get(CRServo.class, "spinTakeRight");
////////        // extend arm servos
////////        leftExtendServo = hardwareMap.get(Servo.class, "leftExtendServo");
////////        rightExtendServo = hardwareMap.get(Servo.class, "rightExtendServo");
////////        // turning arm servos
//////        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
//////        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
////////        // wrist servos
////////        leftWristServo = hardwareMap.get(Servo.class, "leftWristServo");
////////        rightWristServo = hardwareMap.get(Servo.class, "rightWristServo");
////////        // LATCH servo
////////        latchServo = hardwareMap.get(Servo.class, "latchServo");
////////
////////        LATCH_OPEN_POSITION = latchServo.getPosition();
////////        ARM_START_POSITION_LEFT = leftArmServo.getPosition();
////////        ARM_START_POSITION_RIGHT = leftArmServo.getPosition();
//////    }
//////
//////    @Override
//////    public void loop() {
//////        // ---- Drivetrain Control ----
//////        double y = -gamepad1.left_stick_y;  // Forward/backward
//////        double x = gamepad1.left_stick_x * 1.1;  // Strafe
//////        double rx = gamepad1.right_stick_x;  // Rotation
//////
//////        // Calculate motor powers
//////        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//////        double frontLeftPower = (y + x + rx) / denominator;
//////        double backLeftPower = (y - x + rx) / denominator;
//////        double frontRightPower = (y - x - rx) / denominator;
//////        double backRightPower = (y + x - rx) / denominator;
//////
//////        // Set motor powers
//////        backLeft.setPower(backLeftPower);
//////        backRight.setPower(backRightPower);
//////        frontLeft.setPower(frontLeftPower);
//////        frontRight.setPower(frontRightPower);
//////
//////        // ------- Wrist CODE -------
//////        // SPIN TAKE servo code
////////        if (gamepad1.right_trigger > 0) {
////////            spinTakeLeft.setPower(1);
////////            spinTakeRight.setPower(-1);
////////        } else if (gamepad1.left_trigger > 0) {
////////            spinTakeLeft.setPower(-1);
////////            spinTakeRight.setPower(1);
////////        } else {
////////            spinTakeLeft.setPower(0);
////////            spinTakeRight.setPower(0);
////////        }
////////        // CLAW servo
////////        if(gamepad2.right_bumper){
////////            clawServo.setPosition(clawServo.getPosition() + 0.005);
////////        } else if(gamepad2.left_bumper){
////////            clawServo.setPosition(clawServo.getPosition() - 0.005);
////////        }
////////        // WRIST servo code
////////        if(gamepad2.dpad_left){
////////            leftWristServo.setPosition(leftWristServo.getPosition() - .005);
////////            rightWristServo.setPosition(rightWristServo.getPosition() + .005);
////////        }else if(gamepad2.dpad_right){
////////            leftWristServo.setPosition(leftWristServo.getPosition() + .005);
////////            rightWristServo.setPosition(rightWristServo.getPosition() - .005);
////////        }
////////
////////        // -------ARM Code------
////////        // ARM EXTEND Code
////////        if(gamepad1.right_bumper){
////////            // leftExtendServo.setPower(1);
////////            // rightExtendServo.setPower(-1);
////////            leftExtendServo.setPosition(leftExtendServo.getPosition() - .005);
////////            rightExtendServo.setPosition(Math.min(rightExtendServo.getPosition() + .005,Math.abs(1 - leftExtendServo.getPosition() )));
////////        } else if(gamepad1.left_bumper) {
////////            // leftExtendServo.setPower(-1);
////////            // rightExtendServo.setPower(1);
////////            leftExtendServo.setPosition(leftExtendServo.getPosition() + .005);
////////            rightExtendServo.setPosition(Math.max(rightExtendServo.getPosition() - .005, Math.abs(1 - leftExtendServo.getPosition())));
////////        }
////////        telemetry.addData("Left extend servo: ", leftExtendServo.getPosition());
////////        telemetry.addData("Right extend servo: ", rightExtendServo.getPosition());
////////        // ARM servo code with constrains
//////        if (gamepad2.left_trigger > 0) {
//////            leftArmServo.setPosition(leftArmServo.getPosition() + 0.001);
//////            rightArmServo.setPosition(rightArmServo.getPosition() - 0.001);
//////        } else if (gamepad2.right_trigger > 0) {
//////            leftArmServo.setPosition(leftArmServo.getPosition() - 0.001);
//////            rightArmServo.setPosition(rightArmServo.getPosition() - 0.001);
//////        }
//////        telemetry.addData("Left servo arm: ", leftArmServo.getPosition());
//////        telemetry.addData("Right servo arm: ", rightArmServo.getPosition());
//////
//////        // SLIDES movement code
//////        if (gamepad2.left_stick_y < 0 && leftSlideMotor.getCurrentPosition() < 8100 && rightSlideMotor.getCurrentPosition() > -8100) {
//////            rightSlideMotor.setPower(-gamepad2.left_stick_y * -1);
//////            leftSlideMotor.setPower(-gamepad2.left_stick_y * 1);
//////        } else if (gamepad2.left_stick_y > 0 && leftSlideMotor.getCurrentPosition() > 0 && rightSlideMotor.getCurrentPosition() < 0) {
//////            rightSlideMotor.setPower(-gamepad2.left_stick_y * -1);
//////            leftSlideMotor.setPower(-gamepad2.left_stick_y * 1);
//////        } else {
//////            rightSlideMotor.setPower(0);
//////            leftSlideMotor.setPower(0);
//////        }
//////        // Debugging COMMENT LATER ON
//////        if (gamepad2.y) {
//////            rightSlideMotor.setPower(-gamepad2.left_stick_y * -0.2);
//////            leftSlideMotor.setPower(-gamepad2.left_stick_y * 0.2);
//////        }
//////
////////        telemetry.addData("Latch Position", latchServo.getPosition());
////////        if (gamepad1.x) {
////////            latchServo.setPosition(latchServo.getPosition() + .003);
////////        } else if(gamepad1.y) {
////////            latchServo.setPosition(latchServo.getPosition() - .003);
////////        }
//////
//////    }
//////}
////
//
//
//
//
//package org.firstinspires.ftc.teamcode;
//
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.Servo;
//
//
//@TeleOp(name = "ASTeleOp", group = "TeleOp")
//public class TeleOpFirst extends OpMode {
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
//    private Servo leftShoulderServo;
//    private Servo rightShoulderServo;
//    private Servo leftArmServo;
//    private Servo rightArmServo;
//    private Servo clawServo;
//    private Servo spinTakeExtendLeft;
//    private Servo spinTakeExtendRight;
//    private Servo spinTakeRotationLeft;
//    private Servo spinTakeRotationRight;
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
//        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
//        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
//
//
//        leftShoulderServo = hardwareMap.get(Servo.class, "leftShoulderServo");
//        rightShoulderServo = hardwareMap.get(Servo.class, "rightShoulderServo");
//
//
//        spinTakeLeft = hardwareMap.get(CRServo.class, "spinTakeLeft");
//        spinTakeRight = hardwareMap.get(CRServo.class, "spinTakeRight");
//
//
//        spinTakeExtendLeft = hardwareMap.get(Servo.class, "spinTakeExtendLeft");
//        spinTakeExtendRight = hardwareMap.get(Servo.class, "spinTakeExtendRight");
//
//
//        spinTakeRotationRight = hardwareMap.get(Servo.class, "spinTakeRotationRight");
//        spinTakeRotationLeft = hardwareMap.get(Servo.class, "spinTakeRotationLeft");
//
//
//        clawServo = hardwareMap.get(Servo.class, "clawServo");
//    }
//
//
//    @Override
//    public void loop() {
//        // ---- Drivetrain Control ----
//        controlDrivetrain();
//        // ------ Manual Controls ------
//        controlSpinTakeManual();
//        // ------ Control all claw things -----
//        controlClaw();
//        // ----- Control slides -------
//        controlSlides();
//        // ----- update values of servos and motors -----
//        updateAllTelemetry();
//    }
//
//
//    private void changeServoPositionBy(Servo servo, double delta) {
//        servo.setPosition(servo.getPosition() + delta);
//    }
//
//
//    private void updateAllTelemetry() {
//        telemetry.addData("SpinTake rotation servo left: ", spinTakeRotationLeft.getPosition());
//        telemetry.addData("SpinTake rotation servo right: ", spinTakeRotationRight.getPosition());
//        telemetry.addData("Left servo arm: ", leftArmServo.getPosition());
//        telemetry.addData("Right servo arm: ", rightArmServo.getPosition());
//        telemetry.addData("Left Shoulder arm: ", leftShoulderServo.getPosition());
//        telemetry.addData("Right Shoulder arm: ", rightShoulderServo.getPosition());
//        telemetry.addData("Left slide: ", leftSlideMotor.getCurrentPosition());
//        telemetry.addData("Right right: ", rightSlideMotor.getCurrentPosition());
//        telemetry.update();
//    }
//
//
//    private void controlDrivetrain() {
//        double y = -gamepad1.left_stick_y;  // Forward/backward
//        double x = gamepad1.left_stick_x * 1.1;  // Strafe
//        double rx = gamepad1.right_stick_x;  // Rotation
//
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
//
//        // Set motor powers
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
//            rightSlideMotor.setPower(-gamepad2.left_stick_y * -1);
//            leftSlideMotor.setPower(-gamepad2.left_stick_y * 1);
//        } else if (gamepad2.left_stick_y > 0 && leftSlideMotor.getCurrentPosition() > 0 && rightSlideMotor.getCurrentPosition() < 0) {
//            rightSlideMotor.setPower(-gamepad2.left_stick_y * -1);
//            leftSlideMotor.setPower(-gamepad2.left_stick_y * 1);
//        } else {
//            rightSlideMotor.setPower(0);
//            leftSlideMotor.setPower(0);
//        }
//        // Debugging COMMENT LATER ON
//        if (gamepad2.y) {
//            rightSlideMotor.setPower(-gamepad2.left_stick_y * -0.7);
//            leftSlideMotor.setPower(-gamepad2.left_stick_y * 0.7);
//        }
//    }
//
//
//    private void controlSpinTakeManual() {
//        // SpinTake code
//        if (gamepad1.right_bumper) {
//            spinTakeRight.setPower(1);
//            spinTakeLeft.setPower(-1);
//        } else if (gamepad1.left_bumper) {
//            spinTakeRight.setPower(-1);
//            spinTakeLeft.setPower(1);
//        } else {
//            spinTakeRight.setPower(0);
//            spinTakeLeft.setPower(0);
//        }
//        // Rotation of SpinTake
//        if (gamepad2.dpad_left) {
//            changeServoPositionBy(spinTakeRotationLeft, .01);
//            changeServoPositionBy(spinTakeRotationRight, -.01);
//        } else if(gamepad2.dpad_right) {
//            changeServoPositionBy(spinTakeRotationLeft, -.01);
//            changeServoPositionBy(spinTakeRotationRight, .01);
//        }
//
//
//        // Extend for spinTake
//        if (gamepad2.right_trigger > 0) {
//            changeServoPositionBy(spinTakeExtendRight, .01);
//            changeServoPositionBy(spinTakeExtendLeft, -.01);
//        } else if (gamepad1.left_trigger > 0) {
//            changeServoPositionBy(spinTakeExtendRight, -.01);
//            changeServoPositionBy(spinTakeExtendLeft, .01);
//        }
//    }
//
//
//    private void controlClaw() {
//        // CLAW servo
//        if(gamepad2.right_bumper){
//            changeServoPositionBy(clawServo, .001);
//        } else if(gamepad2.left_bumper){
//            changeServoPositionBy(clawServo, -.001);
//        }
//
//
//        // ARM
//        if (gamepad2.left_trigger > 0) {
//            changeServoPositionBy(leftArmServo, .01);
//            changeServoPositionBy(leftArmServo, -.01);
//        } else if (gamepad2.right_trigger > 0) {
//            changeServoPositionBy(leftArmServo, -.01);
//            changeServoPositionBy(leftArmServo, .01);
//        }
//
//
//        // ARM shoulder servo thing
//        if (gamepad2.left_bumper) {
//            changeServoPositionBy(leftShoulderServo, .01);
//            changeServoPositionBy(rightShoulderServo, -.01);
//        } else if (gamepad2.right_bumper) {
//            changeServoPositionBy(leftShoulderServo, -.01);
//            changeServoPositionBy(rightShoulderServo, .01);
//        }
//    }
//
//
//    // Declare a state and timer
//    private enum SpinTakeState { IDLE, ROTATE_INITIAL, EXTEND, ROTATE_FINAL }
//    private SpinTakeState currentState = SpinTakeState.IDLE;
//    private final ElapsedTime timer = new ElapsedTime();
//    private boolean isBPressed = false; // To distinguish between `b` and `x`
//
//
//    public void controlSpinTake() {
//        // check for button presses to initiate actions
//        if (currentState == SpinTakeState.IDLE) {
//            if (gamepad2.b) {
//                isBPressed = true;
//                currentState = SpinTakeState.ROTATE_INITIAL;
//                timer.reset();
//            } else if (gamepad2.x) {
//                isBPressed = false;
//                currentState = SpinTakeState.ROTATE_INITIAL;
//                timer.reset();
//            }
//        }
//
//
//        // handle servo movements
//        switch (currentState) {
//            case ROTATE_INITIAL:
//                spinTakeRotationLeft.setPosition(0);  // change value for rotation
//                spinTakeRotationRight.setPosition(0); // change value for rotation
//
//
//                // Wait for 0.5 seconds
//                if (timer.seconds() > 0.5) {
//                    if (isBPressed) {
//                        currentState = SpinTakeState.EXTEND;
//                    } else {
//                        currentState = SpinTakeState.IDLE; // for `x`, end here
//                    }
//                    timer.reset();
//                }
//                break;
//
//
//            case EXTEND:
//                spinTakeExtendRight.setPosition(0.1); // change value for extension
//                spinTakeExtendLeft.setPosition(0.1);  // change value for extension
//
//
//                // Wait for 0.5 seconds
//                if (timer.seconds() > 0.5) {
//                    currentState = SpinTakeState.ROTATE_FINAL;
//                    timer.reset();
//                }
//                break;
//
//
//            case ROTATE_FINAL:
//                spinTakeRotationLeft.setPosition(0.1); // change value for rotation
//                spinTakeRotationRight.setPosition(0.1); // change value for rotation
//
//
//                // Wait for 0.5 seconds
//                if (timer.seconds() > 0.5) {
//                    currentState = SpinTakeState.IDLE; // set to idle state
//                    timer.reset();
//                }
//                break;
//
//
//            case IDLE:
//                // do nothing while waiting for input
//                break;
//        }
//    }
//
//
//}
//
//
//// Whole code is made by Shreyansh
//
