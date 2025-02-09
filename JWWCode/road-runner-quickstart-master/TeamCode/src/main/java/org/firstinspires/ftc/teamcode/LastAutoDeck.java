// All code is written by me except when mentioned otherwise.
//Copied that same way it is written in the rr.brott.dev website.
// Then you can take these methods and use followTrajectoryAsync(traj1), this will allow for simultaneous robot movements along with drivetrain.

package org.firstinspires.ftc.teamcode;

import android.transition.Slide;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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

@Autonomous(name = "LastAutoDeck", group = "Autonomous")
// @Disabled
public class LastAutoDeck extends LinearOpMode {

    // add action later after looking at the bot
    // deal with existential dread hide it among the vast lines of code so that nobody notices

    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(15,-63.5, Math.toRadians(90.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // init all the non drive train stuff here
        // int increment = 5;

                // .strafeToConstantHeading(new Vector2d(0, -47)) // Hang Specimen
                // .strafeToConstantHeading(new Vector2d(10, -47))
                // // .splineToConstantHeading(new Vector2d(36.50, -24.00), Math.toRadians(90.00))
                // .splineToConstantHeading(new Vector2d(48.00, 0.00), Math.toRadians(90.00))
                // .strafeToConstantHeading(new Vector2d(48, -52.5))
                // .splineToConstantHeading(new Vector2d(58, 0), Math.toRadians(90.00))
                // .strafeToConstantHeading(new Vector2d(58, -49))
                // .turn(-180)
                // .strafeToLinearHeading(new Vector2d(0, -47), Math.toRadians(90))
                // .strafeToLinearHeading(new Vector2d(58, -49), Math.toRadians(-90))
                // .strafeToLinearHeading(new Vector2d(0, -47), Math.toRadians(90))
                // .strafeToLinearHeading(new Vector2d(58, -49), Math.toRadians(-90))
                // .strafeToLinearHeading(new Vector2d(0, -47), Math.toRadians(90))
                // .build();


        public class AutonSequence {
            public Action MoveToChamber{
                    drive.actionBuilder(initialPose)
                    .strafeToConstantHeading(new Vector2d(0,-47))
                    .build();
            }
            
            public Action PushSamples {
                    drive.actionBuilder(new Pose2d(0, -47, Math.toRadians(90))
                    .strafeToConstantHeading(new Vector2d(10, -47))
                    // .splineToConstantHeading(new Vector2d(36.50, -24.00), Math.toRadians(90.00))
                    .splineToConstantHeading(new Vector2d(48.00, 0.00), Math.toRadians(90.00))
                    .strafeToConstantHeading(new Vector2d(48, -52.5))
                    .splineToConstantHeading(new Vector2d(58, 0), Math.toRadians(90.00))
                    .strafeToConstantHeading(new Vector2d(58, -49))// Move
                    .build();
            }
    
            public Action ClawFirstSpec {
                    drive.actionBuilder(new Pose2d(58, -49, Math.toRadians(90)))
                    .turn(Math.toRadians(-180))
                    .build();
            }
    
    
            public Action HangSpec { 
                    drive.actionBuilder(new Pose2d(58, -49, Math.toRadians(-90)))
                    .strafeToLinearHeading(new Vector2d(0, -47), Math.toRadians(90)) // Increment move
                    .build();
            }
    
            public Action ClawSpec {
                    drive.actionBuilder(new Pose2d(0, -47, Math.toRadians(90)))
                    .strafeToLinearHeading(new Vector2d(58, -49), Math.toRadians(-90)) // Increment move
                    .build();
            }
    
            public Action SafeStrafe {
                    drive.actionBuilder(new Pose2d(0, - 47, Math.toRadians(90)))
                    .strafeToConstantHeading(new Vector2d(-5, -47))
                    .build();
            }
        }

        // Action GrabSpecFirstTime = drive.actionBuilder(new Pose2d((24 + (3*increment)), (-47 + (3*increment)), Math.toRadians(-30)))
        //         .strafeToLinearHeading(new Vector2d(55,-55), Math.toRadians(-90))
        //         .strafeToLinearHeading(new Vector2d(0, -47), Math.toRadians(90))
        //         .build();
        // Action GrabSpec = drive.actionBuilder(new Pose2d(0,-47, Math.toRadians(90)))
        //         .strafeToLinearHeading(new Vector2d(55,-55), Math.toRadians(-90))
        //         .build();
        // Action Score = drive.actionBuilder(new Pose2d(55,-55,Math.toRadians(-90)))
        //         .strafeToLinearHeading(new Vector2d(0, -47), Math.toRadians(90))
        //         .build();

        if (isStopRequested()) return;
        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                    new SequentialAction(
                        AutonSequence.MoveToChamber
                        // ArmPosition
                        // WristPosition
                    )
                    AutonSequence.PushSamples
                    new SequentialAction(
                        AutonSequence.ClawFirstSpec
                        //WristPosition
                    )
                    new SequentialAction(
                        AutonSequence.HangSpec
                        // ArmPosition
                        // WristPostion
                    )
                    // for(int i = 0; i < 2; i++) {
                    // new SequentialAction(
                    //     new SequentialAction(
                    //         AutonSequence.ClawSpec
                    //         // WristPosition
                    //     )
                    //     new SequentialAction(
                    //         AutonSequence.HangSpec
                    //         // ArmPosition
                    //         AutonSequence.SafeStrafe
                    //         // WristPostion
                    //     )
                    // }
                )
        ); // Reverse all positions after each Action :)
        for(int i = 0; i < 2; i++) {
            Actions.runBlocking(
                new SequentialAction(
                        new SequentialAction(
                            AutonSequence.ClawSpec
                            // WristPosition
                        )
                        new SequentialAction(
                            AutonSequence.HangSpec
                            // ArmPosition
                            AutonSequence.SafeStrafe
                            // WristPostion
                        )
                )
            );
        }
    }
}
