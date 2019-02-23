package com.evolutionftc.autopilot;


// Copyright (c) 2016-2019 Aedan Cullen and/or Evolution Robotics.


public class AutopilotSegment {
    public String id;
    public String success;
    public String fail;

    public double[] navigationTarget;
    public double orientationTarget;
    public double navigationGain;
    public double orientationGain;
    public double navigationMax;
    public double navigationMin;
    public double orientationMax;
    public boolean useOrientation;

    public boolean useTranslation = true;
    
    public void populateFromOther(AutopilotSegment other) {
        //this.id = other.id;
        //this.success = other.success;
        //this.fail = other.fail;

        this.navigationTarget = other.navigationTarget;
        this.orientationTarget = other.orientationTarget;
        this.navigationGain = other.navigationGain;
        this.orientationGain = other.orientationGain;
        this.navigationMax = other.navigationMax;
        this.navigationMin = other.navigationMin;
        this.orientationMax = other.orientationMax;
        this.useOrientation = other.useOrientation;
        this.useTranslation = other.useTranslation;
    }
}
