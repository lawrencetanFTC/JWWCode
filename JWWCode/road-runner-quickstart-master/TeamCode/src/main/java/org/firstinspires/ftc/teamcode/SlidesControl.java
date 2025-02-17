package org.firstinspires.ftc.teamcode;

/*import static org.firstinspires.ftc.teamcode.SlidesControl.Slides.leftSlideMotor;
import static org.firstinspires.ftc.teamcode.SlidesControl.Slides.rightSlideMotor;*/

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
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


public class SlidesControl implements Runnable {

    private DcMotor leftSlideMotor;
    private DcMotor rightSlideMotor;
    public SlidesControl(HardwareMap hardwareMap) {
        leftSlideMotor = hardwareMap.get(DcMotor.class, "leftSlideMotor");
        rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");

        rightSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);


    }
    public static boolean run = true;

    public static int slidePosition = 0;

    /*public static class Slides {
        public static DcMotor leftSlideMotor;
        public static DcMotor rightSlideMotor;
        public Slides(HardwareMap hardwareMap) {
            leftSlideMotor = hardwareMap.get(DcMotor.class, "leftSlideMotor");
            rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");

            rightSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        }
        public class SlidesLow implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slidePosition = 0;
                return false;
            }
        }

        public Action low() {
            return new SlidesLow();
        }

        public class SlidesHigh implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slidePosition = 8000;
                return false;
            }
        }

        public Action high() {
            return new SlidesHigh();
        }
    }*/
    @Override
    public void run() {
        while (run) {
            leftSlideMotor.setTargetPosition(slidePosition);
            rightSlideMotor.setTargetPosition(slidePosition);

            leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftSlideMotor.setPower(0.8);
            rightSlideMotor.setPower(0.8); // you can probably up the power here
            // when exiting an op-mode remember to set SlidesControl.run = false and it's better if slides are 0'd
        }
    }
}
