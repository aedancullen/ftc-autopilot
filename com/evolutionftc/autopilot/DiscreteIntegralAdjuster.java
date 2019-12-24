package com.evolutionftc.autopilot;

public class DiscreteIntegralAdjuster {

    long timeAtLastTick = -1;
    double desiredAtLastTick;

    double integral;
    double Ki;

    public DiscreteIntegralAdjuster(double Ki) {
        this.Ki = Ki;
    }

    public double adjust(double desired, double actual) {
        long timeNow = System.currentTimeMillis();
        if (desired == 0) {
            integral = 0;
            timeAtLastTick = -1;
        }
        double output = desired;
        if (timeAtLastTick > 0) {
            double error = actual - desiredAtLastTick;
            integral += error * (timeNow - timeAtLastTick) /1000;
            output -= integral * Ki;
        }
        timeAtLastTick = timeNow;
        desiredAtLastTick = desired;
        return output;
    }
}
