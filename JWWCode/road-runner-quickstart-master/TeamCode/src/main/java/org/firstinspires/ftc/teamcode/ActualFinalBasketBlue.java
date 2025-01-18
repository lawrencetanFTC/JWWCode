package org.firstinspires.ftc.teamcode;



import org.firstinspires.ftc.teamcode._0AutonSkeleton.*;

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
public class ActualFinalBasketBlue extends LinearOpMode {
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(-24, 60 , Math.toRadians(90));
        waitForStart();


        Slides slide = new Slides(hardwareMap);
        MecanumDrive drive = new  MecanumDrive(hardwareMap, initialPose);

        Action MoveToBasket = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(53.00, 53.00), Math.toRadians(45))
//                .splineToLinearHeading(new Pose2d(54.50, 54.50, Math.toRadians(45.00)), Math.toRadians(45.00))
                .build();




    }


}
