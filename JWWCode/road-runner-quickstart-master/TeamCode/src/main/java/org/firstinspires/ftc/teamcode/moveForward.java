//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//@Autonomous(name = "ForwardAuto", group = "Autonomous")
//public class moveForward extends LinearOpMode {
//
//    // Motors for driving
//    private DcMotor frontLeft;
//    private DcMotor frontRight;
//    private DcMotor backLeft;
//    private DcMotor backRight;
//
//    @Override
//    public void runOpMode() {
//        // Initialize hardware
//        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
//        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
//        backRight = hardwareMap.get(DcMotor.class, "backRight");
//
//        // Reverse left motors for proper directionality
//        frontLeft.setDirection(DcMotor.Direction.REVERSE);
//        backLeft.setDirection(DcMotor.Direction.REVERSE);
//
//        // Wait for the game to start
//        waitForStart();
//
//        if (opModeIsActive()) {
//            // Move forward
//            moveForward(0.5, 1000); // Power: 0.5, Duration: 1000 ms
//
//            // Stop motors
//            stopMotors();
//        }
//    }
//
//    private void moveForward(double power, int duration) {
//        frontLeft.setPower(power);
//        frontRight.setPower(power);
//        backLeft.setPower(power);
//        backRight.setPower(power);
//
//        sleep(duration); // Wait for the specified duration
//    }
//
//    private void stopMotors() {
//        frontLeft.setPower(0);
//        frontRight.setPower(0);
//        backLeft.setPower(0);
//        backRight.setPower(0);
//    }
//}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "ForwardAuto", group = "Autonomous")
public class moveForward extends LinearOpMode {

    // Motors for driving
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    @Override
    public void runOpMode() {
        // Initialize hardware
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Reverse left motors for proper directionality
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start
        waitForStart();

        if (opModeIsActive()) {
            // Move forward
            moveForward(0.5, 1000); // Power: 0.5, Duration: 1000 ms

            // Strafe left
            strafeLeft(0.5, 1000); // Power: 0.5, Duration: 1000 ms

            // Strafe right
            strafeRight(0.5, 1000); // Power: 0.5, Duration: 1000 ms

            // Move backward
            moveBackward(0.5, 1000); // Power: 0.5, Duration: 1000 ms

            // Stop motors
            stopMotors();
        }
    }

    private void moveForward(double power, int duration) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        sleep(duration); // Wait for the specified duration
    }

    private void moveBackward(double power, int duration) {
        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(-power);

        sleep(duration); // Wait for the specified duration
    }

    private void strafeLeft(double power, int duration) {
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);

        sleep(duration); // Wait for the specified duration
    }

    private void strafeRight(double power, int duration) {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);

        sleep(duration); // Wait for the specified duration
    }

    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}



