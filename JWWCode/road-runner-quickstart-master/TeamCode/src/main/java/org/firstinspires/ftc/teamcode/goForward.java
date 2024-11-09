package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous
public class goForward  extends LinearOpMode {

    private DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
    private DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
    private DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
    private DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");

    MecanumDrive drive = new MecanumDrive( hardwareMap, new Pose2d(0,0 ,0) );

    public class PID {
        DcMotorEx motor;
        double integralSum = 0;
        double KP, KI, KD;
        double lastError = 0;
        ElapsedTime timer = new ElapsedTime();



        private PID(HardwareMap hardwareMap, String deviceName, double kp, double ki, double kd) {
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


    public void runOpMode(){
        Pose2d InitialPose = new Pose2d(0, 0, 0);
        waitForStart();
        if (opModeIsActive()) {;
            // DefaultMotorPower(frontLeft, backLeft, frontRight, backRight);
            Actions.runBlocking(
                drive.ActionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .strafeTo(1,0)
                        .build()
                );
            );
        }
    }

//    public  void DefaultMotorPower(frontLeft, frontRight, backLeft, backRight){
//          frontLeft.setPower(power);
//          frontRight.setPower(power);
//          backLeft.setPower(power);
//          backRight.setPower(power);


    }
}
