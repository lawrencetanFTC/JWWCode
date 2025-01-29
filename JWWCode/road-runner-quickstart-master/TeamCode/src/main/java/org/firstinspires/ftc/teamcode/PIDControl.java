package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class PIDControl{

    public static double error;


    public static int desiredPos = 0;
    public DcMotor dcMotor;
    double esum = 0.0;

    public double kP = 0;
    public double kI = 0;
    public double kD = 0;
    public double lastError = 0;
    public double max = 24;
    public double maxPower = .7;








    public void changeDesiredPos( int desiredPos){
        this.desiredPos = desiredPos;
    };

    PIDControl(DcMotor dcMotor){
        this.dcMotor = dcMotor;
    }

    public void computePID(){
        lastError = error;

        error = dcMotor.getCurrentPosition() - desiredPos;
        esum = esum + error;


        double pTerm = error * kP;
        double iTerm = esum * kI;
        double dTerm = (error - lastError) * kD;


        double ticks = Math.signum(pTerm + iTerm + dTerm) * Math.max(pTerm+ iTerm + dTerm, max);
        double power = ticks / max * maxPower;

        dcMotor.setPower(power);

        // dcMotor.setPosition(ticks)







    }



}
