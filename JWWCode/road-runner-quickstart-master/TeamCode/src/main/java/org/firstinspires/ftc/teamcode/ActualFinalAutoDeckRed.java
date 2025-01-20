package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import org.firstinspires.ftc.teamcode.HelloAuto.*;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "ActualFinalAutoDeck", group = "Autonomous")
public class ActualFinalAutoDeckRed extends LinearOpMode {

    @Override
    public void runOpMode() {

        Extend extend = new Extend(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Spintake spintake = new Spintake(hardwareMap);
        Slides slides = new Slides(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);


        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
        Pose2d initialPose = new Pose2d(-24, 60, Math.toRadians(90));


        Action goToRungs = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(0, -37))
                .build();

        Action park = drive.actionBuilder(new Pose2d(0,-37,Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(0,-40))
                .splineToLinearHeading(new Pose2d(44, -60, Math.toRadians(180)),Math.toRadians(180))
                .build();

		/*
		Hello, this is a short guide for robot positions.
		Just copy and paste the sequence provided below
		for each major position and adjust as needed.
		This should work assuming Autonomous is all good.
		In the near future, when I have the time avaigit lable
		and neccesary, I will write it as an action as the
		doc is a little unclear on whether I can put it as
		'raw code' before running the OpMode or like how I
		wrote this. - Sohan

		P.S - there's probably a lot of spelling errors.

		Starting Position - Run once Auton starts to put
		the robot into a "neutral" state.

		new ParallelAction(
			extend.retract(),
			wrist.up(),
			spintake.neutral(),
			slides.moveToMid(),
			arm.down(),
			claw.close()
		)

		Spintake Position - Run for floor to claw.

		new sequentialAction( //Best to leave it a seq until latter.
			wrist.down(),
			extend.exten(),
			spintake.intake(), //Time
			spintake.neutral(),
			extend.retract(),
			wrist.up(),
			claw.open(),
			arm.down(),
			slides.moveToLow(),
			spintake.outtake(),
			claw.close(),
			arm.up(),
			slides.moveToHigh()
		)

		Specimens - for hanging specimens, first is from wall, second is from ground.

		new sequentialAction(
			claw.close(), //Move to the rungs then continue
			arm.up(),
			slides.moveToMid(),
			slides.hook(),
			claw.open()
		)

		new sequentialAction(
			wrist.down(),
			extend.exten(),
			spintake.intake(), //Time
			spintake.neutral(),
			extend.retract(),
			wrist.up(),
			claw.open(),
			arm.down(),
			slides.moveToLow(),
			spintake.outtake(), //Time
			claw.close(),
			arm.up(),
			slides.moveToMid(), //Wait until finished with traq
			slides.hook(),
			claw.open()
		)

		*/
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        claw.openClaw(),
                        claw.closeClaw(),
                        arm.armUp(),
                        arm.armDown(),
                        slides.basketPos(),
                        slides.rungPos(),
                        slides.hook(),
                        slides.lowPos(),
                        slides.rungPos()
                )
        );

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
    }}

//    public static class Slides {
//        DcMotor leftSlideMotor;
//        DcMotor rightSlideMotor;
//
//        int LOW_POS = 10;  // TO TUNE
//        int MID_POS = 50;
//        int HIGH_POS = 90;
//        int HOOK_DELTA = 5; // move down 5 ticks to hook specimen
//
//        Slides(HardwareMap hardwareMap) {
//            leftSlideMotor = hardwareMap.get(DcMotor.class, "leftSlideMotor");
//            rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");
//
//            leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            leftSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        }
//
//        public Action moveToTop() {
//            return new SetSlidePosition(HIGH_POS);
//        }
//
//        public Action moveToMid() {
//            return new SetSlidePosition(MID_POS);
//        }
//
//        public Action moveToLow() {
//            return new SetSlidePosition(LOW_POS);
//        }
//
//        public Action hook() {
//            return new SetSlidePosition(rightSlideMotor.getCurrentPosition() - HOOK_DELTA);
//        }
//
//        public class SetSlidePosition implements Action {
//
//            private final int position;
//            boolean init = false;
//
//            SetSlidePosition(int position) {
//                this.position = position;
//            }
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                if (!init) {
//                    leftSlideMotor.setPower(.3);
//                    leftSlideMotor.setTargetPosition(-position);
//                    rightSlideMotor.setPower(.3);
//                    rightSlideMotor.setTargetPosition(position);
//                    init = true;
//                }
//
//                boolean running = leftSlideMotor.getCurrentPosition() != -position;
//                if (!running) {
//                    leftSlideMotor.setPower(0);
//                    rightSlideMotor.setPower(0);
//                }
//
//                return running;
//            }
//        }
//    }
//
//    public class Arm {
//        Servo lArmServo;
//        Servo rArmServo;
//
//        Arm(HardwareMap hardwareMap) {
//            lArmServo = hardwareMap.get(Servo.class, "leftArmServo");
//            rArmServo = hardwareMap.get(Servo.class, "rightArmServo");
//        }
//
//
//        // Actions Go here.
//        // NUMBERS IN ARNAV CODE ARE PURPOSFULLY LOW
//        // THESE NUMBERES WILL BE TUNED LATER
//
//        public Action up() {
//            return new ArmVertical();
//        }
//
//        public Action down() {
//            return new ArmHorizontal();
//        }
//
//        //Actions added by arnav
//        public class ArmVertical implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                lArmServo.setPosition(0.5);
//                rArmServo.setPosition(-0.5);
//                return false;
//            }
//        }
//
//        public class ArmHorizontal implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                lArmServo.setPosition(0);
//                rArmServo.setPosition(0);
//                return false;
//            }
//        }
//        // End of arnav code
//
//    }
//
//// Paste slides and arms actioned code here
//
//    public class Claw {
//        private final Servo claw;
//
//        public Claw(HardwareMap hardwareMap) {
//            claw = hardwareMap.get(Servo.class, "clawServo");
//        }
//
//        public Action close() {
//            return new CloseClaw();
//        }
//
//        public Action open() {
//            return new OpenClaw();
//        }
//
//        public class CloseClaw implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//
//                claw.setPosition(0);
//                return false;
//            }
//        }
//
//        public class OpenClaw implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//
//                claw.setPosition(0.1);
//                return false;
//            }
//        }
//    }
//
//
//}
