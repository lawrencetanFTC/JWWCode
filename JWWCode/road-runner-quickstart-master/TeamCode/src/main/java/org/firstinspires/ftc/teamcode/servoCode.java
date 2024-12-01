package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class servoCode extends OpMode {

    Servo leftarmServo;
    Servo rightarmServo;
    Servo leftextendServo;
    Servo rightextendServo;
    double ArmPos = 0;
    double extendPos = 0;

    public void init(){
        leftarmServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightarmServo = hardwareMap.get(Servo.class, "rightArmServo");
        leftextendServo = hardwareMap.get(Servo.class, "leftextendServo");
        rightextendServo = hardwareMap.get(Servo.class, "rightextendServo");
    }

    public void loop(){
       if(gamepad1.x){
           ArmPos = ArmPos + .1;
           leftarmServo.setPosition(-ArmPos);
           rightarmServo.setPosition(ArmPos);

       } else if(gamepad1.y){
           ArmPos = ArmPos - .1;
           leftarmServo.setPosition(-ArmPos);
           rightarmServo.setPosition(ArmPos);
       }

       if(gamepad1.a){
           extendPos = extendPos + .1;
           leftextendServo.setPosition(-extendPos);
           rightextendServo.setPosition(extendPos);
       } else if(gamepad1.b){
           extendPos = extendPos - .1;
           leftextendServo.setPosition(-extendPos);
           rightextendServo.setPosition(extendPos);
       }

    }

}


