package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class PIDControl {

    public int desiredPos = 0; // Desired motor position (in ticks)
    public DcMotor dcMotor;
    private double esum = 0.0;

    public double kP = 0;
    public double kI = 0;
    public double kD = 0;
    private double lastError = 0;
    private final double max = 24; // Max tick output
    private final double maxPower = 0.7; // Max motor power

    private long lastTime = System.nanoTime(); // For dynamic loop timing

    public PIDControl(DcMotor dcMotor) {
        this.dcMotor = dcMotor;
    }

    public void changeDesiredPos(int desiredPos) {
        this.desiredPos = desiredPos;
    }

    public void computePID() {
        long currentTime = System.nanoTime();
        double loopTime = (currentTime - lastTime) / 1e9; // Convert nanoseconds to seconds
        lastTime = currentTime;

        // Calculate error (desired position - current position)
        double error = desiredPos - dcMotor.getCurrentPosition();
        esum += error;

        // Prevent integral windup
        double iMax = max / (kI == 0 ? 1 : kI); // Avoid division by zero
        esum = Math.max(-iMax, Math.min(iMax, esum));

        // PID terms
        double pTerm = error * kP;
        double iTerm = esum * kI;
        double dTerm = ((error - lastError) / loopTime) * kD;

        // Compute output in tick units (capped at max ticks)
        double ticks = Math.signum(pTerm + iTerm + dTerm) * Math.min(Math.abs(pTerm + iTerm + dTerm), max);

        // Scale output to power range (-maxPower to maxPower)
        double power = (ticks / max) * maxPower;
        power = Math.max(-maxPower, Math.min(maxPower, power)); // Clamp power

        dcMotor.setPower(power); // Apply power to motor

        lastError = error; // Update last error
    }
}
