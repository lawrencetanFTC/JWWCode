//
//
//package org.firstinspires.ftc.teamcode;
//
////import com.acmerobotics.roadrunner.control.PIDCoefficients;
////import com.acmerobotics.roadrunner.control.PIDFController;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import java.util.HashMap;
//import java.util.zip.CRC32;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
////import org.firstinspires.ftc.teamcode.IsolatedSlidePID;
////import org.firstinspires.ftc.teamcode.drive.IsolatedSlidePositioning;
////import org.firstinspires.ftc.teamcode.drive.StratagemInput;
//// ^ not sure what type of servo we have
//
//@TeleOp(name="Drive2", group="Linear OpMode")
//
//public class Drive2 extends LinearOpMode {
//    /*
//    // PID constants
//    private final double Kp = 0.01;  // Proportional gain
//    private final double Ki = 0.0;   // Integral gain
//    private final double Kd = 0.001; // Derivative gain
//
//    // PID variables
//    private double targetPosition = 0; // Desired encoder position
//    private double integralSum = 0;
//    private double lastError = 0;
//    */
//
//
//    // Fine adjustment increment
//    private final int TALL_BASKET = 2845; // Small movement increments in encoder ticks
//    private final int LOW_BASKET = 1475; // Larger increments for D-Pad adjustments
//
//    private final int HIGH_RUNG = 1600;
//
//    private final int LOW_RUNG = 1500;
//    private final int INTAKE = 945;
//    private final int START = 50;
//    private DcMotor leftBackDrive; private DcMotor leftFrontDrive; private DcMotor rightBackDrive; private DcMotor rightFrontDrive; private DcMotor planeLauncher;
//    /* Not Needed for Custom Robot #1
//    private Servo leftExtendServo; private Servo rightExtendServo;
//    private Servo leftARotateServo; private Servo rightARotateServo;
//
//    End of Not Needed for Custom Robot #1
//    */
//    private CRServo wristServo;
//
//    private DcMotor pivotMotor; private DcMotor leftSlide; private DcMotor rightSlide;
//    private Servo leftArmServo; private Servo rightArmServo; private Servo rotateServo; private Servo clawServo;
//    private String leftFrontDriveName = "leftFront";
//    private String leftBackDriveName = "leftBack";
//    private String rightFrontDriveName = "rightFront";
//    private String rightBackDriveName = "rightBack";
//
//    /** Robot subSystem ON/OFF flag settings Begin **/
//
//    private boolean driveEnabled = true;
//
//    /** Robot subSystem ON/OFF Flag settings End **/
//
//    /** Not Needed for Custom Robot #2
//     private String leftSlideMainName = "leftSlide";
//     private String rightSlideMainName = "rightSlide";
//     private String leftExtendServoName = "leftExtendServo"; private String rightExtendServoName = "rightExtendServo";
//     private String leftARotateServoName = "leftARotateServo"; private String rightARotateServoName = "rightARotateServo";
//     private DcMotorEx leftSlideMain;
//     private DcMotor rightSlideMain;
//     End of Not Needed for Custom Robot #2 **/
//
//    /**Limits the Drivetrain Speed **/
//    private double powerLimit(double x) {
//        double newVal = 1 * x;
//        double multiplier = 1 + (gamepad1.right_trigger);
//
//        return newVal;
//    }
//    // fbbf
//
//    @Override
//    public void runOpMode() {
//
//        // Constants for slides
//        // moved to op mode loop so we can edit these values depending on pivot position
//        int MAX_POSITION = 3985; // Maximum encoder ticks (full extension)
//        int MIN_POSITION = 0;   // Minimum encoder ticks (fully retracted)
//
//        ElapsedTime deltaTimer = new ElapsedTime();
//        leftFrontDrive  = hardwareMap.get(DcMotor.class, leftFrontDriveName);
//        rightFrontDrive = hardwareMap.get(DcMotor.class, rightFrontDriveName);
//        leftBackDrive = hardwareMap.get(DcMotor.class, leftBackDriveName);
//        rightBackDrive = hardwareMap.get(DcMotor.class, rightBackDriveName);
//        pivotMotor = hardwareMap.get(DcMotor.class, "pivotMotor");
//
//        //leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
//        //rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
//        Thread slidePIDThread = new Thread(new IsolatedSlidePID(hardwareMap));
//        slidePIDThread.start();
//
//
//        //rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
//
//
//        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
//        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
//        rightArmServo.setDirection(Servo.Direction.REVERSE);
//
//        rotateServo = hardwareMap.get(Servo.class, "rotateServo");
//        wristServo = hardwareMap.get(CRServo.class, "wristServo");
//
//        clawServo = hardwareMap.get(Servo.class, "clawServo");
//        //wristServo = hardwareMap.get(CRServo.class, "wristServo");
//        String compensating = "Not compensating";
//        String[] PREINIT_booleanKeys = {"drive_enabled", "slides_enabled", "arm_enabled", "rotate_enabled"};
//        boolean[] PREINIT_booleanValues = {true, true,true, true};
//        HashMap<String, Integer> PREINIT_booleanKeyToIndex = new HashMap<String, Integer>();
//        for (int i = 0; i < PREINIT_booleanKeys.length; i++) {PREINIT_booleanKeyToIndex.put(PREINIT_booleanKeys[i],i);}
//
//        int PREINIT_booleanIndex = 0;
//        double clawPosition = 0.5; // 0.5 closed | 0.7 open
//
//        //
//        StratagemInput submersibleBackIntakeStratagem = new StratagemInput(gamepad2, 2.0, "dpad_down", "dpad_left");
//        StratagemInput specimenBackIntakeStratagem = new StratagemInput(gamepad2, 2.0, "dpad_down", "dpad_down");
//        StratagemInput highBackOuttakeStratagem = new StratagemInput(gamepad2, 2.0, "dpad_up", "dpad_right");
//        while (!opModeIsActive()) {
//
//            if (gamepad1.dpad_down) {
//                while (gamepad1.dpad_down) {}
//                PREINIT_booleanIndex = Math.max(0,Math.min(PREINIT_booleanKeys.length-1, PREINIT_booleanIndex+1));
//            }
//            if (gamepad1.dpad_up) {
//                while (gamepad1.dpad_up) {}
//                PREINIT_booleanIndex = Math.max(0,Math.min(PREINIT_booleanKeys.length-1, PREINIT_booleanIndex-1));
//            }
//            if (gamepad1.dpad_right) {
//                while (gamepad1.dpad_right) {}
//                PREINIT_booleanValues[PREINIT_booleanIndex] = true;
//            }
//            if (gamepad1.dpad_left) {
//                while (gamepad1.dpad_left) {}
//                PREINIT_booleanValues[PREINIT_booleanIndex] = false;
//            }
//            telemetry.addLine("Pre-initialization settings");
//            for (int i = 0; i < PREINIT_booleanKeys.length; i++) {
//                String display = PREINIT_booleanKeys[i];
//                if (i == PREINIT_booleanIndex) {display = "[" + display + "]"; }
//                telemetry.addData(display, PREINIT_booleanValues[i]);
//            }
//            telemetry.addData("mode:", pivotMotor.getMode());
//
//            telemetry.update();
//        }
//
//        waitForStart();
//
//        //pivotMotor.setTargetPosition(0);
//        //pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //pivotMotor.setPower(0.8);
//
//
//        String presetFlag = "";
//        double armServoPosition = 0.36;
//        double rotatePosition = 0.5;
//        int pivotPosition = 0;
//        double wristPosition = 0;
//        while (opModeIsActive()) {
//            double y = -gamepad1.left_stick_y;
//            double x = gamepad1.left_stick_x * 1.1;
//            double rx = gamepad1.right_stick_x;
//
//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//            double leftFrontPower = powerLimit((y + x + rx) / denominator);
//            double leftBackPower = powerLimit((y - x + rx) / denominator);
//            double rightFrontPower = powerLimit((y - x - rx) / denominator);
//            double rightBackPower = powerLimit((y + x - rx) / denominator);
//
//
//            if (PREINIT_booleanValues[PREINIT_booleanKeyToIndex.get("drive_enabled")]) {
//                leftBackDrive.setPower(leftBackPower);
//                rightBackDrive.setPower(rightBackPower);
//                leftFrontDrive.setPower(leftFrontPower);
//                rightFrontDrive.setPower(rightFrontPower);
//            }
//
//
//            if (PREINIT_booleanValues[PREINIT_booleanKeyToIndex.get("slides_enabled")]) {
//                //double currentPosition = (leftSlide.getCurrentPosition() + rightSlide.getCurrentPosition()) / 2.0;
//
//               /*// Calculate PID terms
//               double error = targetPosition - currentPosition;
//               integralSum += error; // Accumulate error for integral term
//               double derivative = error - lastError; // Change in error for derivative term
//
//               // Compute motor power using PID
//               double power = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
//
//               // Apply power to both motors
//               leftSlide.setPower(power);
//               rightSlide.setPower(power);
//
//               // Update last error
//               lastError = error;
//*/
//            }
//            if (PREINIT_booleanValues[PREINIT_booleanKeyToIndex.get("arm_enabled")]) {
//                leftArmServo.setPosition(armServoPosition);
//                rightArmServo.setPosition(armServoPosition);
//                telemetry.addData("arm position", armServoPosition);
//            }
//            if (PREINIT_booleanValues[PREINIT_booleanKeyToIndex.get("rotate_enabled")]) {
//                if (gamepad1.right_bumper) {
//                    while (gamepad1.right_bumper) {}
//                    rotatePosition = clipToBounds(0.0, 1.0, rotatePosition + 0.01);
//                }
//                if (gamepad1.left_bumper) {
//                    while (gamepad1.left_bumper) {}
//                    rotatePosition = clipToBounds(0.1, 1.0, rotatePosition - 0.01);
//                }
//
//                rotateServo.setPosition(rotatePosition);
//                telemetry.addData("rotate servo", rotatePosition);
//            }
//            if (gamepad1.dpad_up) {
//                while (gamepad1.dpad_up) {}
//                IsolatedSlidePID.targetPosition += 100;
//                IsolatedSlidePID.targetPosition = clipToBounds(MIN_POSITION, MAX_POSITION, IsolatedSlidePID.targetPosition);
//
//            }
//            if (gamepad1.dpad_down) {
//                while (gamepad1.dpad_down) { }
//                IsolatedSlidePID.targetPosition -= 100;
//                IsolatedSlidePID.targetPosition = clipToBounds(MIN_POSITION, MAX_POSITION, IsolatedSlidePID.targetPosition);
//            }
//            if (pivotPosition == 0 && pivotMotor.getCurrentPosition()<0 && pivotMotor.getCurrentPosition() > -55) {
//                pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                pivotMotor.setPower(0.008);
//                compensating = "Is compensating";
//            } else {
//                pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                pivotMotor.setTargetPosition(pivotPosition);
//                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                pivotMotor.setPower(1.0);
//                compensating = "Isn't compensating";
//            }
//
//            if (gamepad1.dpad_right) {
//                while (gamepad1.dpad_right) {}
//                pivotPosition += 10;
//            }
//            if (gamepad1.dpad_left) {
//                while (gamepad1.dpad_left) { }
//                pivotPosition -= 100;
//            }
//            double SLIDES_SPEED = 8000;
//
//            if (pivotPosition < -650) {
//                SLIDES_SPEED /= 8;
//                MAX_POSITION = 2500;
//            }
//
//            if (gamepad1.right_trigger > 0.2) {
//                IsolatedSlidePID.targetPosition = clipToBounds(MIN_POSITION, MAX_POSITION, (IsolatedSlidePID.targetPosition + (SLIDES_SPEED * deltaTimer.milliseconds() / 1000)));
//            }
//            if (gamepad1.left_trigger > 0.2) {
//                IsolatedSlidePID.targetPosition = clipToBounds(MIN_POSITION, MAX_POSITION, (IsolatedSlidePID.targetPosition - (SLIDES_SPEED * deltaTimer.milliseconds() / 1000)));
//            }
//            if (gamepad1.x) {
//                armServoPosition -= 0.01;
//            }
//            if (gamepad1.a) {
//                armServoPosition += 0.01;
//            }
//
//
//
//            if (gamepad1.b) {
//                while (gamepad1.b) {}
//                if (presetFlag.equals("submersibleBackIntake")) {
//                    armServoPosition = 0.365;
//                    leftArmServo.setPosition(armServoPosition);
//                    rightArmServo.setPosition(armServoPosition);
//                    sleep(250);
//                    clawPosition = 0.5;
//                    clawServo.setPosition(clawPosition);
//                    sleep(250);
//                    armServoPosition = 0.39;
//                    leftArmServo.setPosition(armServoPosition);
//                    rightArmServo.setPosition(armServoPosition);
//                    rotatePosition = 0.5;
//                    rotateServo.setPosition(0.5);
//                    sleep(100);
//                    wristPosition = -0.575;
//                    presetFlag = "";
//                }
//                else if (clawPosition == 0.5) {clawPosition = 0.7;}
//                else {clawPosition = 0.5;}
//            }
//
//
//            if (gamepad2.dpad_up) {
//                IsolatedSlidePID.targetPosition = 1300;
//                pivotPosition = 0;
//                clawPosition = 0.5;
//                rotatePosition = 0.2;
//                armServoPosition = 0.62;
//                wristPosition = 0.54;
//
//            }
//
//            if (gamepad2.a) {
//                IsolatedSlidePID.targetPosition = 0;
//                pivotPosition = 0;
//                clawPosition = 0.5;
//                rotatePosition = 0.5;
//                armServoPosition = 0.36;
//            }
//
//            /*if (gamepad2.dpad_left) { //specimen intake front
//                armServoPosition = 0.66;
//                rotatePosition = 0.54;
//                IsolatedSlidePID.targetPosition = 0;
//            }
//            */
//
//            if (gamepad2.y) {
//                IsolatedSlidePID.targetPosition = 1885;
//            }
//
//            if (gamepad2.left_bumper) {
//                wristPosition = 0.54;
//            } else if (gamepad2.right_bumper) {
//                wristPosition = -0.575;
//            } else if (gamepad2.x) {
//                wristPosition = 0.0;
//            }
//            if (submersibleBackIntakeStratagem.resolve()) {
//                armServoPosition = 0.39;
//                rotatePosition = 0.1;
//                pivotPosition = -830;
//                clawPosition = 0.7;
//                presetFlag = "submersibleBackIntake";
//                pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                pivotMotor.setTargetPosition(pivotPosition);
//                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                pivotMotor.setPower(0.8);
//            }
//            if (specimenBackIntakeStratagem.resolve()) {
//                armServoPosition = 0.07;
//                pivotPosition = 0;
//                rotatePosition = 0.5;
//                wristPosition = -0.575;
//                IsolatedSlidePID.targetPosition=0;
//                presetFlag = "specimenBackIntake";
//            }
//            if (highBackOuttakeStratagem.resolve()) {
//                armServoPosition = 0.48;
//                rotatePosition = 0.71;
//                IsolatedSlidePID.targetPosition = 3985;
//                clawPosition = 0.5;
//                wristPosition = -0.575;
//            }
//            clawServo.setPosition(clawPosition);
//            IsolatedSlidePID.running = true;
//            wristServo.setPower(wristPosition);
//
//            submersibleBackIntakeStratagem.update(deltaTimer.seconds());
//            specimenBackIntakeStratagem.update(deltaTimer.seconds());
//            highBackOuttakeStratagem.update(deltaTimer.seconds());
//            telemetry.addData("Is compensating?", compensating);
//            telemetry.addData("pivot current position", pivotMotor.getCurrentPosition());
//            telemetry.addData("pivot position", pivotPosition);
//            telemetry.addData("slide position", IsolatedSlidePID.targetPosition);
//            telemetry.addData("claw position", clawPosition);
//            telemetry.addData("arm position", armServoPosition);
//            telemetry.addData("rotate Position", rotatePosition);
//            telemetry.addData("wrist", wristPosition);
//            telemetry.update();
//            deltaTimer.reset();
//
//        }
//
//
//        //IsolatedSlidePositioning.slidePosition = 0;
//        //sleep(3000);
//
//
//    }
//    public double clipToBounds(double min, double max, double edit) {
//        return Math.max(min, Math.min(max, edit));
//
//
//
//
//    }
//
//    public int confineSlideDistance(int ticks) {
//        // TODO: this restricts slides to what we measured as the min/max with some margin
//        // TODO: also nothing to do
//        int MINIMUM = 0;
//        int MAXIMUS = 1400;
//        return Math.max(MINIMUM, Math.min(MAXIMUS, ticks));
//    }
//
//    public double clampSpeed(double manualSpeed) {
//        // TODO: get rid of this since this is only for right trigger slide movement
//        return Math.max(Math.min(manualSpeed, 0.58), -0.05);
//    }
//
//}
//
//
//
