package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.action.Action;
import com.acmerobotics.roadrunner.action.SequentialAction;
import com.acmerobotics.roadrunner.action.ParallelAction;
import com.acmerobotics.roadrunner.action.Actions;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="AutonPath")
import java.util.ArrayList;

public class Drive {
    public Action turn(double angle) {
        return Actions.runBlocking(
                Drive.turnAsync(angle)
    }

    // i & j stand for horizontal & vertical axis respectively...
    public Action moveToPoint(double i, double j) {
        return Actions.runBlocking(
                Drive.actionBuilder(new Pose2d(i, j, angle))
                        .lineToX(i)
                        .build()
        );
    }

    public Action strafeToPoint(double i, double j) {
        return Actions.runBlocking(
                Drive.actionBuilder(new Pose2d(i, j, angle))
                        .strafeTo(i, j)
                        .build()
        );
    }

    public Action splineToPoint(double i, double j, double targetAngle) {
        return Actions.runBlocking(
                Drive.actionBuilder(new Pose2d(i, j, angle))
                        .setTangent(0)
                        .splineTo(i, j, targetAngle)
                        .build()
        );
    }
}

public class ServoControl {
    public Action setPosition(double p) {
        return ServoControl.setPosition(p);
    }
    
    public Action setPower(double p2) {
        return ServoControl.setPower(p2);
    }
}

public class MotorControl {
    public Action setPower(double p) {
        return MotorControl.setPower(p);
    }
}

public class AutonPath extends LinearOpMode{
    public void runOpMode throws InterruptedException{
        Drive pathCreator = new Drive();

        waitForStart();

        Actions.runBlocking(
                // Will Run in Sequence of Operation given
                new SequentialAction(
                pathCreator.turn(angle),
                /* Will Run at the same time;
                therefore, turns can happen at the same time as movement...
                Benefits: Increases Robot Speed & Accuracy, but has to be timed well.
                */
                new SequentialAction(
                        new SequentialAction(
                            /*
                             Movement 1 to 2, can use any condensed form of...
                             splineToPoint, strafeToPoint, etc.
                             Ex: pathCreator.splineToPoint(P1, P2, P3);
                            */
                                new ParallelAction(

                                ) // Motors & Servos, while driving in specific orders
                        ) // Drivetrain
                )
        )
        );
    }
} // Create instances of each class for specific motors, servos, etc.
