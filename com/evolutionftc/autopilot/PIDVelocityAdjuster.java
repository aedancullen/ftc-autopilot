package com.evolutionftc.autopilot;

public class PIDVelocityAdjuster {

    long timeAtLastTick = -1;
    double desiredAtLastTick;
    double outputAtLastTick;
    double deltaDump;

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
        long timeNow = System.nanoTime() / 1000000;

        if (desired == 0) {
            reset();
        }
        double output = desired;
        if (timeAtLastTick > 0) {

            long elapsed = (timeNow - timeAtLastTick) / 1000;
            if (elapsed == 0) { // "zero elapsed time", so ignore this tick (no update)
                // Need to "dump" this delta to not lose it in the magic timewarp
                deltaDump += actualDelta;
                return Math.max(0.0, Math.min(1.0, outputAtLastTick));
            }
            // elapsed != 0, so apply and reset the dump
            actualDelta += deltaDump;
            deltaDump = 0;
            double actual = (actualDelta / elapsed) / actualPeakRate;
            double error = actual - desiredAtLastTick;

            if (Kp != 0) {
                output -= error * Kp;
            }

            if (Ki != 0) {
                integral += error * elapsed;
                integral = Math.max(-desired, Math.min(1.0 - desired, integral * Ki)) / Ki;
                output -= integral * Ki;
            }

            if (output == 0) { output = 0.001; }
        }
        timeAtLastTick = timeNow;
        desiredAtLastTick = desired;
        outputAtLastTick = output;

        return Math.max(0.0, Math.min(1.0, output));
    }
}
