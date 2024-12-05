package org.firstinspires.ftc.teamcode.oldAutos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "BasketAuto", group = "Autonomous")
public class AutonPositionBasket extends LinearOpMode {


    public class PID {
        DcMotorEx motor;
        double integralSum = 0;
        double KP, KI, KD;
        double lastError = 0;
        ElapsedTime timer = new ElapsedTime();


        public PID(HardwareMap hardwareMap, String deviceName, double kp, double ki, double kd) {
            motor = hardwareMap.get(DcMotorEx.class, deviceName);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            KP = kp;
            KI = ki;
            KD = kd;
        }

        public void runOpMode() throws InterruptedException {
            waitForStart();
            timer.reset();
            while (opModeIsActive()) {
                double power = PIDControl(1000, motor.getVelocity());
                motor.setPower(power);
            }
        }


        public double PIDControl(double reference, double state) {
            double error = reference - state;
            integralSum += error * timer.seconds();
            double derivative = (error - lastError) / timer.seconds();
            lastError = error;


            double output = (error * KP) + (integralSum * KI) + (derivative * KD);
            timer.reset(); // Reset timer for the next loop
            return output;
        }


        // Method to set power directly
        public void setPower(double power) {
            motor.setPower(power);
        }


        // Example of instantiation (this should be done in the main op mode)
    }


    // Example instantiation
    PID spintakeMotor = new PID(hardwareMap, "spintakeMotor", 1.0, 0.1, 0.01);
    PID clawMotor = new PID(hardwareMap, "clawMotor", 1.0, 0.1, 0.01);
    PID shoulderMotor = new PID(hardwareMap, "shoulderMotor", 1.0, 0.1, 0.01);




    public class RPServo {
        private Servo rpservo;


        public RPServo(HardwareMap hardwareMap) {
            rpservo = hardwareMap.get(Servo.class, "rpservo");
        }
        // ChatGPT Servo Actions
        public void setPosition(double position) {
            // Position should be between 0.0 (0 degrees) and 1.0 (180 degrees)
            if (position < 0.0 || position > 1.0) {
                throw new IllegalArgumentException("Position must be between 0.0 and 1.0");
            }
            rpservo.setPosition(position);
        }


        // Method to increment the servo position
        public void incrementPosition(double increment) {
            double newPosition = rpservo.getPosition() + increment;


            // Clamp the value to stay within the bounds
            if (newPosition < 0.0) newPosition = 0.0;
            if (newPosition > 1.0) newPosition = 1.0;


            rpservo.setPosition(newPosition);
        }


        // Method to reset the servo to the starting position
        public void resetPosition() {
            rpservo.setPosition(0.0); // Assumes starting position is 0.0
        }


        // Optional: Method to get the current position of the servo
        public double getPosition() {
            return rpservo.getPosition();
        }


        public RPServo spintakeServo = new RPServo(hardwareMap);
        public RPServo clawServo = new RPServo(hardwareMap);
        public RPServo shoulderServo = new RPServo(hardwareMap);
    }


    // Any code regarding calling the .setPower() method or the .setPosition(); .incrementPosition; etc. here
    // Sample Code for clawMotor ~
    // clawMotor.runOpMode() for using PID to set motor power
    // clawMotor.setPower(power) for directly giving the motor an initial/default power




    @Override
    public void runOpMode() {
        // Initialize motors
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");


        // Ensure motors are stopped before the game starts
        stopMotors(frontLeft, backLeft, frontRight, backRight);


        // Starting pose (0, 0, 0)
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);


        waitForStart();


        if (opModeIsActive()) {
            // Set Motor Power after the Game Starts
            DefaultMotorPower(frontLeft, backLeft, frontRight, backRight);


            // Move to Waypoint 1 (from (0, 0) to (-34, -24))
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .strafeTo(new Vector2d(-34, -24)) // Drop off Specimen
                            .build()
            );
            stopMotors(frontLeft, backLeft, frontRight, backRight);
            DefaultMotorPower(frontLeft, backLeft, frontRight, backRight);


//            Actions.runBlocking(
//                    .setTanget(0)
//                    .splineToSplineHeading(Pose2d(20, -6, -45), Math.PI/2)
//            // Move directly to Waypoint 3, & thereafter add Extend Arm Code for collecting Neutral Samples
//           );


            // Move back to Waypoint 2 (34, -24) for TeleOp (Reverse Last Step)
        }
    }


    public void DefaultMotorPower(DcMotor frontLeft, DcMotor backLeft, DcMotor frontRight, DcMotor backRight) {
//        frontLeft.setPower(motorPower);
//        backLeft.setPower(motorPower);
//        frontRight.setPower(motorPower);
//        backRight.setPower(motorPower);
    }


    // Method to stop all drive motors
    public void stopMotors(DcMotor frontLeft, DcMotor backLeft, DcMotor frontRight, DcMotor backRight) {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }
}
