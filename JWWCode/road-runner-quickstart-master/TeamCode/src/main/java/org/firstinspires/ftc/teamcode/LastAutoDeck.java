// All code is written by me except when mentioned otherwise.
//Copied that same way it is written in the rr.brott.dev website.
// According to Priyam, js take the trajectories and put them into different methods such as traj 1, traj 2, etc. depending on which trajectories you want to order in a sequence, which you can put in state
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
        int increment = 5;

        Action deckFull = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(0, -47)) // Hang Specimen
                .strafeToLinearHeading(new Vector2d(24, -47), Math.toRadians(60)) // Move
                .turn(Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d((24 + (2*increment)), (-47 + (2*increment))), Math.toRadians(60)) // Increment move
                .turn(Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d((24 + (3*increment)), (-47 + (3*increment))), Math.toRadians(60)) // Increment move
                .turn(Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(55,-55), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(0, -47), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(55,-55), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(0, -47), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(55,-55), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(0, -47), Math.toRadians(90))
                .build();



        Action MoveToChamber = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(0,-47))
                .build();
        Action MoveToSample = drive.actionBuilder(new Pose2d(0,-37, -Math.PI/2))
                .strafeToLinearHeading(new Vector2d(24, -47), Math.toRadians(60)) // Move
                .build();


        Action sweep = drive.actionBuilder(new Pose2d(24,-24, Math.toRadians(-45)))
                .turn(Math.toRadians(-90))
                .build();


        Action MoveToNext = drive.actionBuilder(new Pose2d(24,-47, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d((24 + (2*increment)), (-47 + (2*increment))), Math.toRadians(60)) // Increment move
                .build();

        Action MoveToThird = drive.actionBuilder(new Pose2d((24 + (2*increment)), (-47 + (2*increment)), Math.toRadians(60)))
                .strafeToLinearHeading(new Vector2d((24 + (3*increment)), (-47 + (3*increment))), Math.toRadians(60)) // Increment move
                .build();


        Action GrabSpecFirstTime = drive.actionBuilder(new Pose2d((24 + (3*increment)), (-47 + (3*increment)), Math.toRadians(-30)))
                .strafeToLinearHeading(new Vector2d(55,-55), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(0, -47), Math.toRadians(90))
                .build();
        Action GrabSpec = drive.actionBuilder(new Pose2d(0,-47, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(55,-55), Math.toRadians(-90))
                .build();

        Action Score = drive.actionBuilder(new Pose2d(55,-55,Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(0, -47), Math.toRadians(90))
                .build();






        Actions.runBlocking(
                new SequentialAction(
                        //stuff that needs to happen on init
                )
        );


        if (isStopRequested()) return;
        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                       // auton goes here
                        deckFull
                )
        );

        //shreynashe dumb or smth idk
    }
}
