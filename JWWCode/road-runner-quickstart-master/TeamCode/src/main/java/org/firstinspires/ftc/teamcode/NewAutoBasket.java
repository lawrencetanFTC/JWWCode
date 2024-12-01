package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import com.acmerobotics.roadrunner.Pose2d;

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

@Autonomous(name = "NewAutoBasket")
public class NewAutoBasket extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
        SpinTake spinTake = new SpinTake(hardwareMap);

        waitForStart();

        TrajectoryActionBuilder oneToTwo = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)));

        Actions.runBlocking(
                oneToTwo.splineToConstantHeading(new Vector2d(10, 10), Math.toRadians(90))
                        .build());

        Actions.runBlocking(spinTake.spinIn(2000));
        Actions.runBlocking(spinTake.spinOut(3000));

        TrajectoryActionBuilder twoToThree = drive.actionBuilder(new Pose2d(10, 10, Math.PI/2));
        Action twoToThreeAction = twoToThree.splineToConstantHeading(new Vector2d(0,5), Math.toRadians(90)).build();
        Actions.runBlocking(twoToThreeAction);


    }

    public class SpinTake {
        CRServo spinTakeServo;

        public SpinTake(HardwareMap hardwareMap) {
            spinTakeServo = hardwareMap.get(CRServo.class, "spintakeservo");
        }

        public Action spinIn(long runMs) {
            return new SpinAction(runMs, DcMotorSimple.Direction.FORWARD);
        }

        public Action spinOut(long runMs) {
            return new SpinAction(runMs, DcMotorSimple.Direction.REVERSE);
        }

        private class SpinAction implements Action {
            private final long runMs;
            private final DcMotorSimple.Direction direction;
            long startTime;

            public SpinAction(long runMs, DcMotorSimple.Direction direction) {
                this.runMs = runMs;
                startTime = 0;
                this.direction = direction;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (startTime == 0) {
                    spinTakeServo.setPower(.5);
                    spinTakeServo.setDirection(direction);
                    startTime = System.currentTimeMillis();
                }
                boolean running = System.currentTimeMillis() - startTime < runMs;
                if (!running) {
                    spinTakeServo.setPower(0);
                }
                return running;
            }
        }
    }
}

