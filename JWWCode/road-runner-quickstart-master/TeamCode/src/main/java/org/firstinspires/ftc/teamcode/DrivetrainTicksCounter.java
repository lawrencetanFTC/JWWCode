//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//@TeleOp(name = "bro at this point idk what this is ", group = "TeleOp")
////@Autonomous(name = "DrivetrainTicksCounter", group = "Autonomous")
//public class DrivetrainTicksCounter extends OpMode {
//
//    // Motors for driving
//    private DcMotor frontLeft;
//    private DcMotor frontRight;
//    private DcMotor backLeft;
//    private DcMotor backRight;
//    @Override
//    public void init() {
//        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
//        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
//        backRight = hardwareMap.get(DcMotor.class, "backRight");
//
//        // Reverse left motors for proper directionality
//        frontLeft.setDirection(DcMotor.Direction.REVERSE);
//        backLeft.setDirection(DcMotor.Direction.REVERSE);
//    }
//
//    @Override
//    public void loop() {
//        // Initialize hardware
//        telemetry.addData("frontLeft: ", frontLeft.getCurrentPosition());
//        telemetry.addData("frontRight: ", frontRight.getCurrentPosition());
//        telemetry.addData("backLeft: ", backLeft.getCurrentPosition());
//        telemetry.addData("backRight: ", backRight.getCurrentPosition());
//        // Move forward
////            moveForward(0.5, 1000); // Power: 0.5, Duration: 1000 ms
////
////            // Strafe left
////            strafeLeft(0.5, 1000); // Power: 0.5, Duration: 1000 ms
////
////            // Strafe right
////            strafeRight(0.5, 1000); // Power: 0.5, Duration: 1000 ms
////
////            // Move backward
////            moveBackward(0.5, 1000); // Power: 0.5, Duration: 1000 ms
////
////            // Stop motors
////            stopMotors();
//
//    }
//}
//
//
//

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Drivetrain Tick Movement", group = "TeleOp")
public class DrivetrainTicksCounter extends OpMode {

    // Motors for driving
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Reverse left motors for proper directionality
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders
        resetEncoders();
    }

    @Override
    public void loop() {
        // Example: Move the drivetrain forward by 1000 ticks at 0.5 power
        moveDrivetrainToPosition(0.5, 1000);

        // Display encoder values
        telemetry.addData("frontLeft: ", frontLeft.getCurrentPosition());
        telemetry.addData("frontRight: ", frontRight.getCurrentPosition());
        telemetry.addData("backLeft: ", backLeft.getCurrentPosition());
        telemetry.addData("backRight: ", backRight.getCurrentPosition());
        telemetry.update();
    }

    /**
     * Moves the drivetrain to a specific tick count.
     *
     * @param power          The power to apply to the motors.
     * @param targetPosition The target encoder position (in ticks).
     */
    private void moveDrivetrainToPosition(double power, int targetPosition) {
        // Set target positions
        frontLeft.setTargetPosition(-targetPosition);
        frontRight.setTargetPosition(targetPosition);
        backLeft.setTargetPosition(-targetPosition);
        backRight.setTargetPosition(targetPosition);

        // Set motors to RUN_TO_POSITION mode
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor power
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        // Wait until all motors reach the target position
        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {
            telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
            telemetry.addData("frontRight", frontRight.getCurrentPosition());
            telemetry.addData("backLeft", backLeft.getCurrentPosition());
            telemetry.addData("backRight", backRight.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motors
        stopMotors();

        // Reset motor mode to RUN_USING_ENCODER
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Stops all drivetrain motors.
     */
    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    /**
     * Resets the motor encoders.
     */
    private void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Checks if the OpMode is active (only relevant for LinearOpMode).
     *
     * @return true if active, false otherwise.
     */
    private boolean opModeIsActive() {
        // Since this is a TeleOp class, we always return true.
        // For Autonomous, you can check if the LinearOpMode is active.
        return true;
    }
}
