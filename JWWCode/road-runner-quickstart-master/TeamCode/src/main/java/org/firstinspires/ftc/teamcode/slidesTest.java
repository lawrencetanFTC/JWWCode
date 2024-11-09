package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class slidesTest extends OpMode {
    DcMotor slideMotorLeft = hardwareMap.get(DcMotor.class, "slideLeft");
    DcMotor slideMotorRight = hardwareMap.get(DcMotor.class, "slideRight");

    @Override
    public void init(){


    }

    @Override
    public void loop(){
        if(gamepad1.a){
            slideMotorLeft.setPower(-.3);
            slideMotorRight.setPower(.3);
        } else if(gamepad1.b){
            slideMotorLeft.setPower(.3);
            slideMotorRight.setPower(-.3);
        } else {
            slideMotorLeft.setPower(0);
            slideMotorRight.setPower(0);
        }



    }
}
