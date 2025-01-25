package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
@Config

public class Team_23344 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "backRight");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()){

            leftFront.setPower(0.5);
            rightBack.setPower(0.5);
            rightFront.setPower(0.5);
            leftBack.setPower(0.5);

            sleep (3000);

            leftFront.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);

        }
    }
}
