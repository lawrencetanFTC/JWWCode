//package org.firstinspires.ftc.teamcode;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//public class hardwareAuto {
//    public class SpinTake {
//        CRServo spinTakeLeft;
//        CRServo spinTakeRight;
//
//        public SpinTake(HardwareMap hardwareMap) {
//            spinTakeLeft = hardwareMap.get(CRServo.class, "spinTakeLeft");
//            spinTakeRight = hardwareMap.get(CRServo.class, "spinTakeRight");
//        }
//
//        private class SpinAction implements Action {
//            private final long runMs;
//            private final DcMotorSimple.Direction direction;
//            long startTime;
//
//            public SpinAction(long runMs, DcMotorSimple.Direction direction) {
//                this.runMs = runMs;
//                startTime = 0;
//                this.direction = direction;
//            }
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                if (startTime == 0) {
//                    spinTakeLeft.setPower(-.5);
//                    spinTakeRight.setPower(.5);
//                    spinTakeLeft.setDirection(direction);
//                    spinTakeRight.setDirection(direction);
//                    startTime = System.currentTimeMillis();
//                }
//                boolean running = System.currentTimeMillis() - startTime < runMs;
//                if (!running) {
//                    spinTakeLeft.setPower(0);
//                    spinTakeRight.setPower(0);
//                }
//                return running;
//            }
//        }
//
//        public Action spinIn(long runMs) {
//            return new  SpinTake.SpinAction(runMs, DcMotorSimple.Direction.FORWARD);
//        }
//    }
//
//    public class ArmServos {
//        Servo lArmServo;
//        Servo rArmServo;
//
//        ArmServos(HardwareMap hardwareMap) {
//            lArmServo = hardwareMap.get(Servo.class, "leftArmServo");
//            rArmServo = hardwareMap.get(Servo.class, "rightArmServo");
//        }
//
//
//        // Actions Go here.
//        // NUMBERS IN ARNAV CODE ARE PURPOSFULLY LOW
//        // THESE NUMBERES WILL BE TUNED LATER
//
//        //Actions added by arnav
//        public  class ArmVertical implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                lArmServo.setPosition(0.5);
//                rArmServo.setPosition(-0.5);
//                return false;
//            }
//        }
//        public Action armUp() {
//            return new  ArmServos.ArmVertical();
//        }
//
//        public  class ArmHorizontal implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                lArmServo.setPosition(0);
//                rArmServo.setPosition(0);
//                return false;
//            }
//        }
//        public Action armDown() {
//            return new  ArmServos.ArmHorizontal();
//        }
//        // End of arnav code
//
//    }
//
//    public static class Slide {
//        static DcMotor leftSlideMotor;
//        static DcMotor rightSlideMotor;
//
//        int LOW_POS = 10;  // TO TUNE
//        int MID_POS = 50;
//        int HIGH_POS = 90;
//        int HOOK_DELTA = 5; // move down 5 ticks to hook specimen
//
//        Slide(HardwareMap hardwareMap) {
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
//        public static class SetSlidePosition implements Action {
//
//            private final int position;
//            boolean init = false;
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
//
//            SetSlidePosition(int position) {
//                this.position = position;
//            }
//        }
//
//        public Action moveToTop() {
//            return new  Slide.SetSlidePosition(HIGH_POS);
//        }
//
//        public Action moveToMid() {
//            return new  Slide.SetSlidePosition(MID_POS);
//        }
//
//        public Action moveToLow() {
//            return new  Slide.SetSlidePosition(LOW_POS);
//        }
//
//        public Action hook() {
//            return new  Slide.SetSlidePosition(rightSlideMotor.getCurrentPosition() - HOOK_DELTA);
//        }
//    }
//
//    public class Claw{
//        Servo clawServo;
//        Claw(HardwareMap hardwareMap){
//            clawServo = hardwareMap.get(Servo.class, "clawServo");
//        }
//
//        double OPEN_POS = .2; // TO TUNE
//        double CLOSE_POS = .6;
//
//        class SetClawPos implements Action{
//
//            private final double position;
//            boolean init = false;
//
//            SetClawPos(double position){
//                this.position = position;
//            }
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                if(!init){
//                    clawServo.setPosition(position);
//                    init = true;
//                }
//
//                return clawServo.getPosition() != position;
//            }
//        }
//        public Action open(){
//            return new  Claw.SetClawPos(OPEN_POS);
//        }
//
//        public Action close(){
//            return new  Claw.SetClawPos(CLOSE_POS);
//        }
//    }
//
//    // Code that Arnav added
//    public class Wrist {
//        Servo leftWristServo;
//        Servo rightWristServo;
//
//        public Wrist(HardwareMap hardwareMap) {
//            leftWristServo = hardwareMap.get(Servo.class, "leftWristServo");
//            rightWristServo = hardwareMap.get(Servo.class, "rightWristServo");
//        }
//
//        public class WristVertical implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                leftWristServo.setPosition(0.3);
//                rightWristServo.setPosition(-0.3);
//                return false;
//            }
//        }
//        public Action wristUp() {
//            return new  Wrist.WristVertical();
//        }
//
//        public class WristHorizontal implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                leftWristServo.setPosition(0);
//                rightWristServo.setPosition(0);
//                return false;
//            }
//        }
//        public Action wristDown() {
//            return new  Wrist.WristHorizontal();
//        }
//
//    }
//
//    public class Drop {
//        Servo dropServo;
//
//        public Drop(HardwareMap hardwaremap) {
//            dropServo = hardwaremap.get(Servo.class, "dropServo");
//        }
//
//        public class OpenDrop implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                dropServo.setPosition(0.1);
//                return false;
//            }
//        }
//        public Action dropSample() {return new  Drop.OpenDrop();}
//
//        public class CloseLatch implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                dropServo.setPosition(0);
//                return false;
//            }
//        }
//        public Action closeDrop() {return new Drop.CloseLatch();}
//    }
//
//    public class Extend {
//        Servo leftExtendServo;
//        Servo rightExtendServo;
//
//        public Extend(HardwareMap hardwaremap) {
//            leftExtendServo = hardwaremap.get(Servo.class, "leftExtendServo");
//            rightExtendServo = hardwaremap.get(Servo.class, "rightExtendServo");
//        }
//
//        public class ResetExtend implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                leftExtendServo.setPosition(0);
//                rightExtendServo.setPosition(0);
//                return false;
//            }
//        }
//        public Action reset() {return new Extend.ResetExtend();}
//
//        public class FullExtend implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                leftExtendServo.setPosition(0.1);
//                rightExtendServo.setPosition(0.1);
//                return false;
//            }
//        }
//        public Action full() {return new Extend.FullExtend();}
//
//        public class NudgeExtend implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                leftExtendServo.setPosition(leftExtendServo.getPosition() + 0.01);
//                rightExtendServo.setPosition(rightExtendServo.getPosition() + 0.01);
//                return false;
//            }
//        }
//        public Action nudge() {return new Extend.NudgeExtend();}
//    }
//}
