package com.evolutionftc.autopilot;


// Copyright (c) 2016-2018 Aedan Cullen and/or Evolution Robotics.


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

    public void populateFromOther(AutopilotSegment other) {
        //this.id = other.id;
        //this.success = other.success;
        //this.fail = other.fail;

        this.navigationTarget = other.navigationTarget;
        this.orientationTarget = other.orientationTarget;
        this.steeringGain = other.steeringGain;
        this.accuracyThreshold = other.accuracyThreshold;
        this.orientationThreshold = other.orientationThreshold;
        this.basePower = other.basePower;
        this.lowestPower = other.lowestPower;
        this.powerGain = other.powerGain;
        this.rampUp = other.rampUp;
        this.rampDown = other.rampDown;
        this.useOrientation = other.useOrientation;
    }
}
