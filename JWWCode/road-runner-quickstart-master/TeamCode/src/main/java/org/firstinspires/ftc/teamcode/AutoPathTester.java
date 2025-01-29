//package org.firstinspires.ftc.teamcode;
//
//import androidx.annotation.NonNull;
//
//// RR-specific imports
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Actions;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.Vector2d;
//
//// Non-RR imports
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//
//@Config
//@Autonomous(name = "AutoPathTest", group = "Autonomous")
//public class AutoPathTester extends LinearOpMode {
//    @Override
//    public void runOpMode() {
//        Pose2d initialPose = new Pose2d(24, -60, Math.toRadians(90));
//        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
//
//        TrajectoryActionBuilder path = drive.actionBuilder(initialPose)
//                .strafeToConstantHeading(new Vector2d(0,-37))
//                .strafeToConstantHeading(new Vector2d(35,-37))
//                .strafeToConstantHeading(new Vector2d(35,-13))
//                .splineToLinearHeading(new Pose2d(45, -13, Math.toRadians(0)), Math.toRadians(-90))
//                .strafeTo(new Vector2d(45,-45))
//                .strafeTo(new Vector2d(45,-13))
//                .strafeToConstantHeading(new Vector2d(55, -13))
//                .strafeTo(new Vector2d(55,-45))
//                .strafeToLinearHeading(new Vector2d(48, -45), Math.toRadians(45))
//                .turn(Math.toRadians(-90))
//                .strafeToLinearHeading(new Vector2d(10, -60), Math.toRadians(0))
//                .strafeToLinearHeading(new Vector2d(0, -37), Math.toRadians(90))
//                .strafeToLinearHeading(new Vector2d(10, -60), Math.toRadians(0))
//                .strafeToLinearHeading(new Vector2d(0, -37), Math.toRadians(90))
//                .strafeToLinearHeading(new Vector2d(10, -60), Math.toRadians(0))
//                .strafeToLinearHeading(new Vector2d(0, -37), Math.toRadians(90));
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        Actions.runBlocking(
//                new ParallelAction(
//                        path.build()
//                )
//        );
//        )
//    }
//}
