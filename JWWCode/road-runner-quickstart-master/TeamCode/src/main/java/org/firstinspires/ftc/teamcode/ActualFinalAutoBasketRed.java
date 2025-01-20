package org.firstinspires.ftc.teamcode;



import org.firstinspires.ftc.teamcode.HelloAuto.*;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous
public class ActualFinalAutoBasketRed extends LinearOpMode {
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(-24, 60 , Math.toRadians(-90));
        waitForStart();



        Slides slide = new Slides(hardwareMap);
        MecanumDrive drive = new  MecanumDrive(hardwareMap, initialPose);
        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        Action MoveToBasket = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(46.92, 50.89), Math.toRadians(0.00))
                .splineToSplineHeading(new Pose2d(54.50, 54.50, Math.toRadians(45.00)), Math.toRadians(45.00))
                .build();




        Actions.runBlocking(new SequentialAction(
                MoveToBasket,
                slide.basketPos(),
                arm.armUp(),
                claw.openClaw(),
                claw.closeClaw(),
                arm.armDown(),
                slide.lowPos()


                ));



    }


}
