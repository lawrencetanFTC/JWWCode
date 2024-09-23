package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class MecanumDrive extends OpMode {

    // Define motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor hangLeft; //left planetary motor on slides
    private DcMotor hangRight; //right planetary motor on slides
    private DcMotor idkMotor; //Motor in the back with slides connected to it

    // Define servos
    private Servo clawServo; // picks up the box
    private Servo holdServo; // holds the sample before hang
    private Servo extendServo; // extends the arm
    private Servo armServo; // moves the arm up and down

    // Servo positions for extendServo and armServo
    private double extendServoPosition = 0.0; // starts fully retracted
    private double armServoPosition = 0.0; // starts fully lowered
    private final double SERVO_INCREMENT = 0.05; // the incremental change when button is held

    @Override
    public void init() {
        // Initialize the motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Initialize the servos
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        extendServo = hardwareMap.get(Servo.class, "extendServo");
        armServo = hardwareMap.get(Servo.class, "armServo"); // added armServo initialization
    }

    @Override
    public void loop() {
        // mecanum drive logic
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double frontLeftPower = y + x + turn;
        double frontRightPower = y - x - turn;
        double backLeftPower = y - x + turn;
        double backRightPower = y + x - turn;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        // ALL SERVO CODE IS ASSUMNIG THAT THE POSITIONS IN THE CAD MODEL
        // ARE ALREADY CALIBRATED TO POSITION 0.00
        // MAKE SURE THE BUILDING TEAM CALIBRATES THIS WHILE BUILDING

        if (gamepad1.a) {
            clawServo.setPosition(0.5); // open the servo to catch box
        } else if (gamepad1.b) {
            clawServo.setPosition(0.0); // close the servo to hold box
        }

        if (gamepad1.x) {
            holdServo.setPosition(0.5); // open the servo to catch sample
        } else if (gamepad1.y) {
            holdServo.setPosition(0.0); // close the servo to hold sample
        }

        if (gamepad2.a) {
            // extend the arm
            extendServoPosition += SERVO_INCREMENT;
            if (extendServoPosition > 1.0) {
                extendServoPosition = 1.0;

            } else if (gamepad2.b) {
                // retract the arm
                extendServoPosition -= SERVO_INCREMENT;
                if (extendServoPosition < 0.0) {
                    extendServoPosition = 0.0;
                }
            }

            extendServo.setPosition(extendServoPosition);


            if (gamepad2.x) {
                // move arm up
                armServoPosition += SERVO_INCREMENT;
                if (armServoPosition > 1.0) {
                    armServoPosition = 1.0;
                }
            } else if (gamepad2.y) {
                // move arm down
                armServoPosition -= SERVO_INCREMENT;
                if (armServoPosition < 0.0) {
                    armServoPosition = 0.0;
                }
            }
            armServo.setPosition(armServoPosition);
        }
    }
