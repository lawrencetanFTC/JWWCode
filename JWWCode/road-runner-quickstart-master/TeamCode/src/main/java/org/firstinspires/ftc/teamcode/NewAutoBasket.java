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
        Slide slide = new Slide(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        waitForStart();

        TrajectoryActionBuilder oneToTwo = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)));

        Actions.runBlocking(
                oneToTwo.splineToConstantHeading(new Vector2d(10, 10), Math.toRadians(90))
                        .build());

        TrajectoryActionBuilder twoToThree = drive.actionBuilder(new Pose2d(10, 10, Math.PI / 2));
        Action twoToThreeAction = twoToThree.splineToConstantHeading(new Vector2d(0, 5), Math.toRadians(90)).build();
        Actions.runBlocking(twoToThreeAction);

        Actions.runBlocking(slide.moveToMid());
        Actions.runBlocking(slide.hook());
        Actions.runBlocking(claw.open());
        Actions.runBlocking(slide.moveToLow());
        Actions.runBlocking(SpinTake.spin);


    }

    public class SpinTake {
        CRServo spinTakeLeft;
        CRServo spinTakeRight;

        public SpinTake(HardwareMap hardwareMap) {
            spinTakeLeft = hardwareMap.get(CRServo.class, "spinTakeLeft");
            spinTakeRight = hardwareMap.get(CRServo.class, "spinTakeRight");
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
                    spinTakeLeft.setPower(.5);
                    spinTakeLeft.setDirection(direction);
                    startTime = System.currentTimeMillis();
                }
                boolean running = System.currentTimeMillis() - startTime < runMs;
                if (!running) {
                    spinTakeLeft.setPower(0);
                }
                return running;
            }
        }

        public Action spinIn(long runMs) {
            return new SpinAction(runMs, DcMotorSimple.Direction.FORWARD);
        }
    }

    public class ArmServos {
        Servo lArmServo;
        Servo rArmServo;

        ArmServos(HardwareMap hardwareMap) {
            lArmServo = hardwareMap.get(Servo.class, "leftArmServo");
            rArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        }


        // Actions Go here.
        // Please code this.

    }

    public class Slide {
        DcMotor leftSlideMotor;
        DcMotor rightSlideMotor;

        int LOW_POS = 10;  // TO TUNE
        int MID_POS = 50;
        int HIGH_POS = 90;
        int HOOK_DELTA = 5; // move down 5 ticks to hook specimen

        Slide(HardwareMap hardwareMap) {
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
            return new SetSlidePosition(HIGH_POS);
        }

        public Action moveToMid() {
            return new SetSlidePosition(MID_POS);
        }

        public Action moveToLow() {
            return new SetSlidePosition(LOW_POS);
        }

        public Action hook() {
            return new SetSlidePosition(rightSlideMotor.getCurrentPosition() - HOOK_DELTA);
        }
    }

    public class Claw{
        Servo clawServo;
        Claw(HardwareMap hardwareMap){
            clawServo = hardwareMap.get(Servo.class, "clawServo");
        }

        double OPEN_POS = .2; // TO TUNE
        double CLOSE_POS = .6;

        class SetClawPos implements Action{

            private final double position;
            boolean init = false;

            SetClawPos(double position){
                this.position = position;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!init){
                    clawServo.setPosition(position);
                    init = true;
                }

                return clawServo.getPosition() != position;
            }
        }
        public Action open(){
            return new SetClawPos(OPEN_POS);
        }

        public Action close(){
            return new SetClawPos(CLOSE_POS);
        }
    }

    // Code that Arnav added
    public class Wrist {
        Servo leftWristServo;
        Servo rightWristServo;

        public Wrist(HardwareMap hardwareMap) {
            leftWristServo = hardwareMap.get(Servo.class, "leftWristServo");
            rightWristServo = hardwareMap.get(Servo.class, "rightWristServo");
        }

        public class WristVertical implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftWristServo.setPosition(1);
                rightWristServo.setPosition(-1);
                return false;
            }
        }
        public Action wristUp() {
            return new WristVertical();
        }

        public class WristHorizontal implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftWristServo.setPosition(0);
                rightWristServo.setPosition(0);
                return false;
            }
        }
        public Action wristDown() {
            return new WristHorizontal();
        }

    }

    public class Drop {
        Servo dropServo;

        public Drop(Hardwaremap hardwaremap) {
            dropServo = hardwaremap.get(Servo.class, "dropServo")
        }

        public class OpenDrop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                dropServo.setPosition(1);
                return false;
            }
        }
        public Action dropSample() {return new OpenDrop();}

        public class CloseDrop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                dropServo.setPosition(0);
                return false;
            }
        }
        public Action closeDrop() {return new CloseDrop();}
    }

}

