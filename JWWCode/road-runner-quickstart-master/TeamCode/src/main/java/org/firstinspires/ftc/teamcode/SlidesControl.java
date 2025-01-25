package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SlidesControl implements Runnable {
    private DcMotor leftSlideMotor; private DcMotor rightSlideMotor;
    public SlidesControl(HardwareMap hardwareMap) {
        leftSlideMotor = hardwareMap.get(DcMotor.class, "leftSlideMotor");
        rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");

        rightSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);


    }
    public static boolean run = true;
    public static int slidePosition = 8000;
    @Override
    public void run() {
        while (run) {
            leftSlideMotor.setTargetPosition(slidePosition);
            rightSlideMotor.setTargetPosition(slidePosition);

            leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftSlideMotor.setPower(0.5);
            rightSlideMotor.setPower(0.5); // you can probably up the power here
            // when exiting an op-mode remember to set SlidesControl.run = false and it's better if slides are 0'd
        }
    }
}
