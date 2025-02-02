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

        Action deckFull = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(5, -49), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(35.5, -42), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(47, -14), Math.toRadians(-45.00))
                .strafeToConstantHeading(new Vector2d(47, -53))
                .splineToConstantHeading(new Vector2d(58, -14), Math.toRadians(-45.00))
                .strafeToConstantHeading(new Vector2d(58, -53))
                .splineToConstantHeading(new Vector2d(40.5, -58.5), Math.toRadians(-90.00))
                .splineToConstantHeading(new Vector2d(2, -48), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(2.5, -55.5), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(40.5, -58), Math.toRadians(-90.00))
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
