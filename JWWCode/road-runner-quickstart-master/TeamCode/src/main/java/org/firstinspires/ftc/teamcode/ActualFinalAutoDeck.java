package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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

@Autonomous(name = "ActualFinalAutoDeck", group = "Autonomous")
public class AcutalFinalAutoDeck extends LinearOpMode {

    public class Arm {
        Servo lArmServo;
        Servo rArmServo;

        Arm(HardwareMap hardwareMap) {
            lArmServo = hardwareMap.get(Servo.class, "leftArmServo");
            rArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        }


        // Actions Go here.
        // NUMBERS IN ARNAV CODE ARE PURPOSFULLY LOW
        // THESE NUMBERES WILL BE TUNED LATER

        //Actions added by arnav
        public class ArmVertical implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                lArmServo.setPosition(0.5);
                rArmServo.setPosition(-0.5);
                return false;
            }
        }
        public Action up() {
            return new  ArmServos.ArmVertical();
        }

        public class ArmHorizontal implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                lArmServo.setPosition(0);
                rArmServo.setPosition(0);
                return false;
            }
        }
        public Action down() {
            return new  ArmServos.ArmHorizontal();
        }
        // End of arnav code

    }

    public static class Slides {
        DcMotor leftSlideMotor;
        DcMotor rightSlideMotor;

        int LOW_POS = 10;  // TO TUNE
        int MID_POS = 50;
        int HIGH_POS = 90;
        int HOOK_DELTA = 5; // move down 5 ticks to hook specimen

        Slides(HardwareMap hardwareMap) {
            leftSlideMotor = hardwareMap.get(DcMotor.class, "leftSlideMotor");
            rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");

            leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class SetSlidePosition implements Action {

            private final int position;
            boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!init) {
                    leftSlideMotor.setPower(.3);
                    leftSlideMotor.setTargetPosition(-position);
                    rightSlideMotor.setPower(.3);
                    rightSlideMotor.setTargetPosition(position);
                    init = true;
                }

                boolean running = leftSlideMotor.getCurrentPosition() != -position;
                if (!running) {
                    leftSlideMotor.setPower(0);
                    rightSlideMotor.setPower(0);
                }

                return running;
            }

            SetSlidePosition(int position) {
                this.position = position;
            }
        }

        public Action moveToTop() {
            return new  Slide.SetSlidePosition(HIGH_POS);
        }

        public Action moveToMid() {
            return new  Slide.SetSlidePosition(MID_POS);
        }

        public Action moveToLow() {
            return new  Slide.SetSlidePosition(LOW_POS);
        }

        public Action hook() {
            return new  Slide.SetSlidePosition(rightSlideMotor.getCurrentPosition() - HOOK_DELTA);
        }
    }


    public class Claw {
        Private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, “clawServo”);
        }

        public class CloseClaw implements Action {
            @Override
            Public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0);
            }
        }

        public Action close() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            Public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.1);
            }
        }

        public Action open() {
            return new OpenClaw();
        }
    }

// Paste slides and arms actioned code here

    @Override
    public void runOpMode() {
        Claw claw = new Claw(hardwareMap);
        Slides slides = new Slides(hardwareMap);
        Arm arm = new Arm(hardwareMap);

        // Trajectoires go here
        // Gonna make these in RRPathgen later

        TrajectoryActionBuilder goToRungs = drive.actionBuilder(new Pose2d(16.00, -63.00, Math.toRadians(90.00)))
                .splineToConstantHeading(new Vector2d(6.00, -34.00), Math.toRadians(90.00))

        TrajectoryActionBuilder pushSamples = drive.actionBuilder(new Pose2d(7.17, -34.00, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(37.00, -34.00))
                .splineToConstantHeading(new Vector2d(37.72, -18.91), Math.toRadians(64.50))
                .splineToConstantHeading(new Vector2d(48.16, -11.27), Math.toRadians(78.26))
                .lineToConstantHeading(new Vector2d(48.00, -56.00))
                .splineToConstantHeading(new Vector2d(48.90, -21.52), Math.toRadians(75.57))
                .splineToConstantHeading(new Vector2d(58.21, -5.12), Math.toRadians(78.65))
                .lineToConstantHeading(new Vector2d(58.40, -56.54))
                .splineToSplineHeading(new Pose2d(63.24, -26.73, Math.toRadians(0.00)), Math.toRadians(72.99))

        TrajectoryActionBuilder goToDeck = drive.actionBuilder(new Pose2d(63.24, -26.73, Math.toRadians(0.00)))
                .lineToSplineHeading(new Pose2d(57.10, -56.72, Math.toRadians(-90.00)))




        if (isStopRequested()) return;

        // Action Test
        Actions.runBlocking(
                new SequntialAction(
                        claw.open(),
                        claw.close(),
                        arm.up(),
                        arm.down(),
                        slides.moveToTop(),
                        slides.moveToMid(),
                        slides.hook(),
                        slides.moveToLow(),
                        goToRungs()
                )
        )

        /*Actions.runBlocking(
                new SequentialAction(
                        claw.close(),
                        slides.moveToMid(),
                        arm.up(),
                        goToRungs.build(),
                        slides.hook(),
                        claw.open(),
                        arm.down(),
                        slides.moveToLow(),
                        pushSamples.build(),
                        claw.close(),
                        goToDeck.build(),
                        claw.open()
                )
        );*/
    }



}
