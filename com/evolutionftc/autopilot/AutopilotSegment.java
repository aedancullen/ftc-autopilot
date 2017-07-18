package com.evolutionftc.autopilot;


// this is a snazzy autopilot for ftc made by aedan.
// it was made somewhat for team #9867, 'evolution'
// it uses lots of maths.
// it is gpl licensed.
// copyright 2017 aedan cullen.


public class AutopilotSegment {
    public int id;
    public int success;
    public int fail;

    public double[] navigationTarget;
    public double steeringGain;
    public double[] accuracyThreshold;
    public double basePower;
    public double lowestPower;
    public double powerGain;
    public boolean rampUp;
    public boolean rampDown;
}
