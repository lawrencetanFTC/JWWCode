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
                extendLeft.setPosition(.2928);
                extendRight.setPosition(.5567);
                return false;
            }
        }
        public Action extend() {
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
        public Action retract() {
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
        public Action stop() {
            return new Neutral();
        }
    }
    public static class Slides {
        public Slides() {}
        public class EditSlidePositions implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return false;
            }
        }
        public Action editSlidePositions(int newSlidePosition) {
            SlidesControl.slidePosition = newSlidePosition;
            return new EditSlidePositions();
        }
    }
    /*public class Slides { //This class is altered from Arnav's code in "ActualFinalAutoDeck.java"
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

        public class Basket implements Action {
            private boolean init = false;
            private int slideBasketPos = 90;
            private double motorPower = 0.3;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!init) {
                    leftSlideMotor.setPower(motorPower);
                    leftSlideMotor.setTargetPosition(slideBasketPos);
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
        public Action basket() {
            return new Basket();
        }

        public class BasketPos implements Action {
            private boolean init = false;
            private int slideBasketPos = 90;
            private double motorPower = 0.3;

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
            return new BasketPos();
        }

        public class RungPos implements Action {
            private boolean init = false;
            private int slideBasketPos = 50;
            private double motorPower = 0.3;

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
        public Action rungPos() {
            return new RungPos();
        }

        public class LowPos implements Action {
            private boolean init = false;
            private int slideBasketPos = 8000;
            private double motorPower = 0.75;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (true) {
                    leftSlideMotor.setPower(motorPower);
                    rightSlideMotor.setPower(motorPower);
                    leftSlideMotor.setTargetPosition(slideBasketPos);
                    rightSlideMotor.setTargetPosition(slideBasketPos);
                    init = true;
                }

                boolean running = leftSlideMotor.getCurrentPosition() != slideBasketPos;
                if (!running) {
                    leftSlideMotor.setPower(0);
                    rightSlideMotor.setPower(0);
                }
                return running;
            }
        }
        public Action lowPos() {
            return new LowPos();
        }

        public class Hook implements Action {
            private boolean init = false;
            private int slideBasketPos = 90;
            private int hookDelta = 5;
            private double motorPower = 0.3;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!init) {
                    leftSlideMotor.setPower(motorPower);
                    leftSlideMotor.setTargetPosition(-(leftSlideMotor.getCurrentPosition() - hookDelta));
                    rightSlideMotor.setPower(motorPower);
                    rightSlideMotor.setTargetPosition(rightSlideMotor.getCurrentPosition() - hookDelta);
                    init = true;
                }

                boolean running = leftSlideMotor.getCurrentPosition() != -(slideBasketPos - hookDelta);
                if (!running) {
                    leftSlideMotor.setPower(0);
                    rightSlideMotor.setPower(0);
                }
                return running;
            }
        }
        public Action hook() {
            return new Hook();
        }
        public class EditSlidePositions implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return false;
            }
        }
        public Action editSlidePositions(int newSlidePosition) {
            SlideControl.slidePosition = newSlidePosition;
            return new EditSlidePositions();
        }

    }*/

    //Major testing below.
//    public static class Slides { //This class is altered from Arnav's code in "ActualFinalAutoDeckBlue.java"
//        private DcMotor leftSlideMotor;
//        private DcMotor rightSlideMotor;
//
//        public Slides(HardwareMap hardwareMap) {
//            leftSlideMotor = hardwareMap.get(DcMotor.class, "leftSlideMotor");
//            rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");
//
//            leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//            leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//            leftSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        }
//
//        public class MoveSlide implements Action {
//            private boolean init = false;
//            private int slideLeftPos = 30;
//            private int slideRightPos = 30;
//            private double motorPower = 0.3;
//
//            MoveSlide(int posLeft, int posRight){
//                this.slideLeftPos = posLeft;
//                this.slideRightPos = posRight;
//            }
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!init) {
//                    leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
////                    leftSlideMotor.setTargetPosition(8473);
////                    rightSlideMotor.setTargetPosition(-8528);
//
//                    leftSlideMotor.setTargetPosition(slideLeftPos);
//                    rightSlideMotor.setTargetPosition(slideRightPos);
//
//                    leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                    leftSlideMotor.setPower(motorPower);
//                    rightSlideMotor.setPower(motorPower);
//
//                    init = true;
//                }
//
//                boolean running = leftSlideMotor.getCurrentPosition() != slideLeftPos;
//                if (!running) {
//                    leftSlideMotor.setPower(0);
//                    rightSlideMotor.setPower(0);
//                }
//                return running;
//            }
//        }
//        // Left is positive right is negative
//
//        // Tune these values to the max slides can extend
//        public Action basketPos() {
//            return new MoveSlide(8473, -8528);
//        }
//
//        // Tune these to the position for hunging specimen
//        public Action rungPos() {
//            return new MoveSlide(500, -500);
//        }
//
//        // Thuse these to the lowest slides can return
//        public Action lowPos() {
//            return new MoveSlide(30, -30);
//        }
//
//        // Tune this so that samples get hooked
//        private int hookDelta = 30;
//
//
//        public Action hook() {
//            return new MoveSlide(rightSlideMotor.getCurrentPosition() + hookDelta, leftSlideMotor.getCurrentPosition() - hookDelta);
//        }
//
//    }

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

        public class ArmRest implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                shoulderLeft.setPosition(0);
                shoulderRight.setPosition(1);
                elbowLeft.setPosition(.9822);
                elbowRight.setPosition(.26);
                return false;
            }
        }

        public Action rest() {return  new ArmRest();}

        public class ArmDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                elbowLeft.setPosition(.9756);
                elbowRight.setPosition(.2844);
                return false;
            }
        }

        public Action down() {return new ArmDown();}

        public class ArmSample implements Action {
            @Override
            public  boolean run(@NonNull TelemetryPacket packet) {
                elbowLeft.setPosition(.5211);
                elbowRight.setPosition(.7661);
                shoulderLeft.setPosition(.89);
                shoulderRight.setPosition(0);
                return false;
            }
        }

        public Action sample() {return new ArmSample();}

        public class ArmSpecimenHang implements Action {
            @Override
            public  boolean run(@NonNull TelemetryPacket packet) {
                shoulderLeft.setPosition(.7767);
                shoulderRight.setPosition(.2217);
                elbowLeft.setPosition(.3611);
                elbowRight.setPosition(.8672);
                return false;
            }
        }

        public Action specimenHang() {return new ArmSpecimenHang();}

        public class ArmSpecimenGrab implements Action {
            @Override
            public  boolean run(@NonNull TelemetryPacket packet) {
                shoulderLeft.setPosition(0);
                shoulderRight.setPosition(1);
                elbowLeft.setPosition(.4761);
                elbowRight.setPosition(.7462);
                return false;
            }
        }

        public Action specimenGrab() {return new ArmSpecimenGrab();}


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

            Extend extend = new Extend(hardwareMap);
            Wrist wrist = new Wrist(hardwareMap);
            Spintake spintake = new Spintake(hardwareMap);
            Slides slides = new Slides();
            Thread slideThread = new Thread(new SlidesControl(hardwareMap));
            slideThread.start();

            Arm arm = new Arm(hardwareMap);
            Claw claw = new Claw(hardwareMap);

            //Create Trajectories here
            Action magic = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(0, -35), Math.toRadians(116.89))
                .strafeToConstantHeading(new Vector2d(0, -40))
                .splineToConstantHeading(new Vector2d(35.5, -40.5), Math.toRadians(1.90))
                .splineToConstantHeading(new Vector2d(37.5, -19), Math.toRadians(56.92))
                .splineToConstantHeading(new Vector2d(43.5, -12.5), Math.toRadians(35.34))
                .strafeToConstantHeading(new Vector2d(43.5, -55.5))
                .splineToConstantHeading(new Vector2d(47.5, -15), Math.toRadians(66.34))
                .splineToConstantHeading(new Vector2d(60, -13), Math.toRadians(78.12))
                .strafeToConstantHeading(new Vector2d(60, -55.5))
                .splineTo(new Vector2d(41.5, -45), Math.toRadians(187.57))
                .splineToLinearHeading(new Pose2d(27, -59, Math.toRadians(0.00)), Math.toRadians(-17.94))
                .strafeToConstantHeading(new Vector2d(32, -59))
                .splineToLinearHeading(new Pose2d(3, -35, Math.toRadians(90.00)), Math.toRadians(150.36))
                .strafeToConstantHeading(new Vector2d(3, -40))
                .splineToLinearHeading(new Pose2d(27, -59, Math.toRadians(0.00)), Math.toRadians(248.96))
                .strafeToConstantHeading(new Vector2d(32, -59))
                .splineToLinearHeading(new Pose2d(3, -35, Math.toRadians(90.00)), Math.toRadians(150.36))
                .strafeToConstantHeading(new Vector2d(3, -40))
                .splineToLinearHeading(new Pose2d(27, -59, Math.toRadians(0.00)), Math.toRadians(248.96))
                .strafeToConstantHeading(new Vector2d(32, -59))
                .splineToLinearHeading(new Pose2d(3, -35, Math.toRadians(90.00)), Math.toRadians(150.36))
                .strafeToConstantHeading(new Vector2d(3, -40))
                .splineToConstantHeading(new Vector2d(40, -59), Math.toRadians(248.96))
                .build();

            /*Action magic2 = drive.actionBuilder(new Pose2d(2, -41, Math.toRadians(90)))
                    h.toRadians(224.46))
                    .build();*/
            //Init Actions here

        Action MoveToBasket = drive.actionBuilder(initialPose)
                .splineToSplineHeading(new Pose2d(52, 52, Math.toRadians(45.00)), Math.toRadians(45.00))
                .build();

        Action MoveToSample1 = drive.actionBuilder(new Pose2d(53,53,Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(48,52), Math.toRadians(-90))
                .build();

        Action Score1 = drive.actionBuilder(new Pose2d(48,51, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(52,52), Math.toRadians(45.00))
                .build();

        Action MoveToSample2 = drive.actionBuilder(new Pose2d(53,53, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(58,45), Math.toRadians(-90))
                .build();

        Action Score2 = drive.actionBuilder(new Pose2d(52,51, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(52,52), Math.toRadians(45.00))
                .build();

        Action MoveToSample3 = drive.actionBuilder(new Pose2d(53,53, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(48, 27), Math.toRadians(0))
                .build();

        Action Score3 = drive.actionBuilder(new Pose2d(47,27, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(53.00, 53.00), Math.toRadians(45))
                .build();

        Action MoveToSubmersible = drive.actionBuilder(new Pose2d(53,53, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(25.00, 5, Math.toRadians(180.00)), Math.toRadians(150.00))
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        claw.closeClaw(),
                        claw.openClaw(),
                        arm.rest(),
                        extend.extend(),
                        wrist.wristUp(),
                        spintake.stop()//                        slides.lowPos()
                )
        );

        // FULL ACTIONS TEST
        // Everything should be fin just tune slide positions before testing
        // IF something is weird have shreyansh look at all the numbers

        // THIS IS AN ACTIONS TEST
        // IT DOESN"T GRAB SAMPLES OR SPECIMENS
        if (isStopRequested()) return;
        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        //magic
                        arm.down(),
                        slides.editSlidePositions(8000),
//                        slides.rungPos(),
                        arm.specimenHang(),
//                        slides.basketPos(),
                        arm.sample(),
                        claw.closeClaw(),
                        claw.openClaw(),
                        arm.specimenGrab(),
                        arm.rest()// arm.rest
//                        slides.lowPos()
                )
        );

//
//        Actions.runBlocking(new SequentialAction(
//                new SequentialAction(
//                        MoveToBasket,
//                        MoveToSample1,
//                        Score1,
//                        MoveToSample2,
//                        Score2,
//                        MoveToSample3,
//                        Score3,
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
