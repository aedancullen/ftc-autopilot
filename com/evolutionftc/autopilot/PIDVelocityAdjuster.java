package com.evolutionftc.autopilot;

public class PIDVelocityAdjuster {

    long timeAtLastTick = -1;
    double desiredAtLastTick;

    double integral;
    double Kp;
    double Ki;
    double Kd;
    double actualPeakRate;

    public PIDVelocityAdjuster(double Kp, double Ki, double Kd, double actualPeakRate) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.actualPeakRate = actualPeakRate;
    }

    public void reset() {
        integral = 0;
        timeAtLastTick = -1;
    }

    public double adjust(double desired, double actualDelta) {
        long timeNow = System.currentTimeMillis();

        if (desired == 0) {
            reset();
        }
        double output = desired;
        if (timeAtLastTick > 0) {

            long elapsed = (timeNow - timeAtLastTick) / 1000;
            double actual = (actualDelta / elapsed) / actualPeakRate;
            double error = actual - desiredAtLastTick;

            output -= error * Kp;

            integral += error * elapsed;
            integral = Math.max(-desired, Math.min(1.0-desired, integral));
            output -= integral * Ki;

            if (output == 0) { output = 0.001; }
        }
        timeAtLastTick = timeNow;
        desiredAtLastTick = desired;

        return Math.max(0.0, Math.min(1.0, output));
    }
}
