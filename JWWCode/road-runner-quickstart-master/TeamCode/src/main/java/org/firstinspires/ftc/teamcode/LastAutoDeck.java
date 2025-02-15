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

        Pose2d initialPose = new Pose2d(15, -63, Math.toRadians(-90.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // init all the non drive train stuff here


        Action SamplePath = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(15, -55), Math.toRadians(104.20))
                .splineToConstantHeading(new Vector2d(0, -33.5), Math.toRadians(90.00))
                .strafeToConstantHeading(new Vector2d(2, -40))
                /*.splineToLinearHeading(new Pose2d(34, -38, Math.toRadians(90.00)), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(46, -14.5), Math.toRadians(-90.00))
                .splineToConstantHeading(new Vector2d(46, -52), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(50, -14), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(57, -14), Math.toRadians(-90.00))
                .splineToConstantHeading(new Vector2d(57, -52), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(41, -61, Math.toRadians(-90.00)), Math.toRadians(-90.00))
                .splineToConstantHeading(new Vector2d(41, -53), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(2, -33.5), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(41, -61), Math.toRadians(-90.00))
                .splineToConstantHeading(new Vector2d(41, -53), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(2, -33.5), Math.toRadians(90.00))*/
                .splineToConstantHeading(new Vector2d(41, -61), Math.toRadians(-90.00))
                .splineToConstantHeading(new Vector2d(41, -53), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(2, -33.5), Math.toRadians(90.00))
                //.strafeToConstantHeading(new Vector2d(0, -33.5))
                .splineToConstantHeading(new Vector2d(41, -61), Math.toRadians(-90.00))
                .build();

        Action GoToRung = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(15, -55), Math.toRadians(104.20))
                .splineToConstantHeading(new Vector2d(0, -33.5), Math.toRadians(90.00))
                .build();

        Action PUSH = drive.actionBuilder(new Pose2d(0, -33.5, Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(2, -40))
                .splineToLinearHeading(new Pose2d(34, -38, Math.toRadians(90.00)), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(46, -14.5), Math.toRadians(-90.00))
                .splineToConstantHeading(new Vector2d(46, -52), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(50, -14), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(57, -14), Math.toRadians(-90.00))
                .splineToConstantHeading(new Vector2d(57, -52), Math.toRadians(-90))
                .build();

        Action back = drive.actionBuilder(new Pose2d(0, -33.5, Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(2, -40))
                .build();

        Action PickSpecimen1 = drive.actionBuilder(new Pose2d(57, -52, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(41, -61, Math.toRadians(-90.00)), Math.toRadians(-90.00))
                .build();

        Action PickSpecimen2 = drive.actionBuilder(new Pose2d(2, -33.5, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(41, -61), Math.toRadians(-90.00))
                .build();

        Action PickSpecimen3 = drive.actionBuilder(new Pose2d(2, -40, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(41, -61), Math.toRadians(-90.00))
                .build();

        Action GoToRung1 = drive.actionBuilder(new Pose2d(41, -61, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(41, -53), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(2, -33.5), Math.toRadians(90.00))
                .build();

        Action park = drive.actionBuilder(new Pose2d(2, -33.5, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(41, -61), Math.toRadians(-90.00))
                .build();

        if (isStopRequested()) return;
        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        // 2 Specimen
                        GoToRung,
                        back,
                        PickSpecimen3,
                        GoToRung1,
                        park
                        /*// 4 Specimen
                        GoToRung,
                        PUSH,
                        PickSpecimen1,
                        GoToRung1,
                        PickSpecimen2,
                        GoToRung1,
                        PickSpecimen2,
                        GoToRung1,
                        park*/
                )
        );

        /*
        Actions.runBlocking(
                new SequentialAction(
                    new SequentialAction(
                        MoveToChamber
                        // ArmPosition
                        // WristPosition
                    ),
                    PushSamples,
                    new SequentialAction(
                        ClawFirstSpec
                        //WristPosition
                    ),
                    new SequentialAction(
                        HangSpec
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
                            ClawSpec
                            // WristPosition
                        ),
                        new SequentialAction(
                            HangSpec,
                            // ArmPosition
                            SafeStrafe
                            // WristPosition
                        )
                )
            );


        }*/
    }
}
