package com.evolutionftc.autopilot;

public class DiscreteIntegralAdjuster {

    long timeAtLastTick = -1;
    double desiredAtLastTick;

    double integral;
    double Ki;

    public DiscreteIntegralAdjuster(double Ki) {
        this.Ki = Ki;
    }

    public double adjust(double desired, double actualDelta, double actualPeakRate) {
        long timeNow = System.currentTimeMillis();

        if (desired == 0) {
            integral = 0;
            timeAtLastTick = -1;
        }
        double output = desired;
        if (timeAtLastTick > 0) {

            long elapsed = (timeNow - timeAtLastTick) / 1000;
            double actual = (actualDelta / elapsed) / actualPeakRate;

            double error = actual - desiredAtLastTick;
            integral += error * elapsed;
            output -= integral * Ki;
        }
        timeAtLastTick = timeNow;
        desiredAtLastTick = desired;
        return output;
    }
}
