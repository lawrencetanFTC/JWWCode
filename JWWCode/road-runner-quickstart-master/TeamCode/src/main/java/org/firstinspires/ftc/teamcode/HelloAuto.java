// All code is written by me except when mentioned otherwise.
//Copied that same way it is written in the rr.brott.dev website.
// According to Priyam, js take the trajectories and put them into different methods such as traj 1, traj 2, etc. depending on which trajectories you want to order in a sequence, which you can put in state
// Then you can take these methods and use followTrajectoryAsync(traj1), this will allow for simultaneous robot movements along with drivetrain.

package org.firstinspires.ftc.teamcode;

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

@Autonomous(name = "AutonSkeleton", group = "Autonomous")
// @Disabled
public class HelloAuto extends LinearOpMode {

    public static class Extend {
        private Servo extendLeft;
        private Servo extendRight;

        public Extend(HardwareMap hardwareMap) {
            extendLeft = hardwareMap.get(Servo.class, "leftExtend");
            extendRight = hardwareMap.get(Servo.class, "rightExtend");
        }

        public class ExtendSt implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                extendLeft.setPosition(.3944);
                extendRight.setPosition(.6028);
                return false;
            }
        }
        public Action extendSt() {
            return new ExtendSt();
        }

        public class RetractSt implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                extendLeft.setPosition(1);
                extendRight.setPosition(0);
                return false;
            }
        }
        public Action retractSt() {
            return new RetractSt();
        }
    }

    public static class Wrist {
        private Servo wristLeft;
        private Servo wristRight;

        public Wrist(HardwareMap hardwareMap) {
            wristLeft = hardwareMap.get(Servo.class, "wristLeft");
            wristRight = hardwareMap.get(Servo.class, "wristRight");
        }

        public class WristUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wristLeft.setPosition(0);
                wristRight.setPosition(1);
                return false;
            }
        }
        public Action wristUp() {
            return new WristUp();
        }

        public class WristDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wristLeft.setPosition(.9317);
                wristRight.setPosition(.0667);
                return false;
            }
        }
        public Action wristDown() {
            return new WristDown();
        }
    }

    public static class Spintake {
        private CRServo spinTakeLeft;
        private CRServo spinTakeRight;

        public Spintake(HardwareMap hardwareMap) {
            spinTakeLeft = hardwareMap.get(CRServo.class, "spinTakeLeft");
            spinTakeRight = hardwareMap.get(CRServo.class, "spinTakeRight");
        }

        public class SpinAction implements Action {
            private DcMotorSimple.Direction direction;

            SpinAction(DcMotorSimple.Direction direction){
                this.direction = direction;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                spinTakeLeft.setDirection(direction);
                spinTakeRight.setDirection(direction);
                spinTakeLeft.setPower(-1);
                spinTakeRight.setPower(1);
                return false;
            }
        }

        public Action intake() {
            return new SpinAction(DcMotorSimple.Direction.FORWARD);
        }

        public Action outtake() {
            return new SpinAction(DcMotorSimple.Direction.REVERSE);
        }

        public class Neutral implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                spinTakeLeft.setPower(0);
                spinTakeRight.setPower(0);
                return false;
            }
        }
        public Action neutral() {
            return new Neutral();
        }
    }

    //Major testing below.
    public static class Slides { //This class is altered from Arnav's code in "ActualFinalAutoDeckBlue.java"
        private DcMotor leftSlideMotor;
        private DcMotor rightSlideMotor;

        public Slides(HardwareMap hardwareMap) {
            leftSlideMotor = hardwareMap.get(DcMotor.class, "leftSlideMotor");
            rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");

            leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            leftSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class MoveSlide implements Action {
            private boolean init = false;
            private int slideBasketPos = 90;
            private double motorPower = 0.3;

            MoveSlide(int pos){
                this.slideBasketPos = pos;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!init) {
                    leftSlideMotor.setPower(motorPower);
                    leftSlideMotor.setTargetPosition(-slideBasketPos);
                    rightSlideMotor.setPower(motorPower);
                    rightSlideMotor.setTargetPosition(slideBasketPos);
                    init = true;
                }

                boolean running = leftSlideMotor.getCurrentPosition() != -slideBasketPos;
                if (!running) {
                    leftSlideMotor.setPower(0);
                    rightSlideMotor.setPower(0);
                }
                return running;
            }
        }
        public Action basketPos() {
            return new MoveSlide(90);
        }

        public Action rungPos() {
            return new MoveSlide(50);
        }


        public Action lowPos() {
            return new MoveSlide(10);
        }
        private int hookDelta = 5;


        public Action hook() {
            return new MoveSlide(rightSlideMotor.getCurrentPosition() - hookDelta);
        }

    }

    public static class Arm {
        private Servo elbowLeft;
        private Servo elbowRight;
        private Servo shoulderLeft;
        private Servo shoulderRight;

        public Arm(HardwareMap hardwareMap) {
            elbowLeft = hardwareMap.get(Servo.class, "elbowLeft");
            elbowRight = hardwareMap.get(Servo.class, "elbowRight");
            shoulderLeft = hardwareMap.get(Servo.class, "shoulderLeft");
            shoulderRight = hardwareMap.get(Servo.class, "shoulderRight");
        }

        public class ArmDown implements Action { //CHECK THIS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                elbowLeft.setPosition(.4489);
                elbowRight.setPosition(.8389);
                shoulderLeft.setPosition(.89);
                shoulderRight.setPosition(0);
                return false;
            }
        }
        public Action armDown() {
            return new ArmDown();
        }

        public class ArmUp implements Action { //CHECK THIS!!!!!!!!!!!!!!!!!!!!!!
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                elbowLeft.setPosition(.9594);
                elbowRight.setPosition(.3033);
                shoulderLeft.setPosition(0);
                shoulderRight.setPosition(1);
                return false;
            }
        }
        public Action armUp() {
            return new ArmUp();
        }
    }

    public static class Claw {
        private Servo clawServo;

        public Claw(HardwareMap hardwareMap) {
            clawServo = hardwareMap.get(Servo.class, "clawServo");
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawServo.setPosition(.6794);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawServo.setPosition(1);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }
    }


    @Override
    public void runOpMode() {

            Pose2d initialPose = new Pose2d(15,-63.5, Math.toRadians(90.00)); //WHAT IS THIS?
            MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

//            Extend extend = new Extend(hardwareMap);
//            Wrist wrist = new Wrist(hardwareMap);
//            Spintake spintake = new Spintake(hardwareMap);
//            Slides slides = new Slides(hardwareMap);
//            Arm arm = new Arm(hardwareMap);
//            Claw claw = new Claw(hardwareMap);

            //Create Trajectories here
            Action magic = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(0, -35), Math.toRadians(116.89))
                .strafeToConstantHeading(new Vector2d(0, -40))
                .splineToConstantHeading(new Vector2d(35.5, -40.5), Math.toRadians(1.90))
                .splineToConstantHeading(new Vector2d(37.5, -19), Math.toRadians(56.92))
                .splineToConstantHeading(new Vector2d(43.5, -12.5), Math.toRadians(35.34))
                .splineToConstantHeading(new Vector2d(43.5, -55.5), Math.toRadians(-89.00))
                .splineToConstantHeading(new Vector2d(47.41, -15.18), Math.toRadians(66.34))
                .splineToConstantHeading(new Vector2d(59.33, -12.95), Math.toRadians(78.12))
                .splineToConstantHeading(new Vector2d(60.45, -55.42), Math.toRadians(-88.19))
                .splineTo(new Vector2d(41.45, -44.80), Math.toRadians(187.57))
                .splineToLinearHeading(new Pose2d(27.29, -58.96, Math.toRadians(0.00)), Math.toRadians(-17.94))
                .splineToConstantHeading(new Vector2d(31.76, -59.15), Math.toRadians(-2.39))
                .splineToLinearHeading(new Pose2d(3.45, -33.81, Math.toRadians(90.00)), Math.toRadians(150.36))
                .splineToConstantHeading(new Vector2d(5.12, -40.14), Math.toRadians(-69.00))
                .splineToLinearHeading(new Pose2d(27.10, -59.15, Math.toRadians(0.00)), Math.toRadians(248.96))
                .splineToConstantHeading(new Vector2d(31.95, -59.52), Math.toRadians(-52.96))
                .build();

            /*Action magic2 = drive.actionBuilder(new Pose2d(2, -41, Math.toRadians(90)))
                    h.toRadians(224.46))
                    .build();*/
            //Init Actions here

        Action MoveToBasket = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(53.00, 53.00), Math.toRadians(45))
                .build();

        Action MoveToSample1 = drive.actionBuilder(new Pose2d(53,53,Math.toRadians(45)))
                .turn(Math.toRadians(-135))
                .strafeTo(new Vector2d(48, 51))
                .build();

        Action PickUpandScore1 = drive.actionBuilder(new Pose2d(48,51, Math.toRadians(-90)))
                .strafeTo(new Vector2d(48, 48))
                .splineTo(new Vector2d(53.00, 53.00), Math.toRadians(45))
                .build();

        Action MoveToSample2 = drive.actionBuilder(new Pose2d(53,53, Math.toRadians(45)))
                .turn(Math.toRadians(-135))
                .strafeTo(new Vector2d(58,51))
                .build();

        Action PickUpandScore2 = drive.actionBuilder(new Pose2d(52,51, Math.toRadians(-90)))
                .strafeTo(new Vector2d(58, 48))
                .splineTo(new Vector2d(53.00, 53.00), Math.toRadians(45))
                .build();

        Action MoveToSample3 = drive.actionBuilder(new Pose2d(53,53, Math.toRadians(45)))
                .turn(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(47,27, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Action PickUpandScore3 = drive.actionBuilder(new Pose2d(47,27, Math.toRadians(0)))
                .strafeTo(new Vector2d(48, 27))
                .splineTo(new Vector2d(53.00, 53.00), Math.toRadians(45))
                .build();

        Action MoveToSubmersible = drive.actionBuilder(new Pose2d(53,53, Math.toRadians(45)))
                .splineTo(new Vector2d(52.00, 45.00), Math.toRadians(238.03))
                .splineToLinearHeading(new Pose2d(25.00, 13.00, Math.toRadians(180.00)), Math.toRadians(180.00))
                .build();

            if (isStopRequested()) return;

            Actions.runBlocking(
                    new SequentialAction(
                            /*magic,
                            new SleepAction(0.1),
                            magic2*/
                            MoveToBasket,
                            MoveToSample1,
                            PickUpandScore1,
                            MoveToSample2,
                            PickUpandScore2,
                            MoveToSample3,
                            PickUpandScore3,
                            MoveToSubmersible
                    )
            );

//
//        Actions.runBlocking(new SequentialAction(
//                new SequentialAction(
//                        MoveToBasket,
//                        MoveToSample1,
//                        PickUpandScore1,
//                        MoveToSample2,
//                        PickUpandScore2,
//                        MoveToSample3,
//                        PickUpandScore3,
//                        MoveToSubmersible
//                )
//
//
//
//
//        ));
//                            new SequentialAction(
//                                    new SleepAction(3),
//                                    extend.extendSt(),
//                                    extend.retractSt(),
//                                    extend.extendSt(),
//                                    wrist.wristUp(),
//                                    wrist.wristDown(),
//                                    spintake.intake(),
//                                    spintake.outtake(),
//                                    spintake.neutral(),
//                                    slides.basketPos(),
//                                    slides.lowPos(),
//                                    slides.rungPos(),
//                                    slides.hook(),
//                                    arm.armDown(),
//                                    arm.armUp(),
//                                    arm.armDown(),
//                                    claw.openClaw(),
//                                    claw.closeClaw()
//                            )
//                    )
        }
}

