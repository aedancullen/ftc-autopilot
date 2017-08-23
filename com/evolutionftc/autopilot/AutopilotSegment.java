package com.evolutionftc.autopilot;


// this is a snazzy autopilot for ftc made by aedan.
// it was made somewhat for team #9867, 'evolution'
// it uses lots of maths.
// copyright 2017 aedan cullen.


public class AutopilotSegment {
    public String id;
    public String success;
    public String fail;

    public double[] navigationTarget;
    public double orientationTarget;
    public double steeringGain;
    public double[] accuracyThreshold;
    public double orientationThreshold;
    public double basePower;
    public double lowestPower;
    public double powerGain;
    public boolean rampUp;
    public boolean rampDown;
    public boolean useOrientation;
}
