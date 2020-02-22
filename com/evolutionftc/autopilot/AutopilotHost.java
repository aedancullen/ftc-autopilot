

package com.evolutionftc.autopilot;



import android.util.Log;

import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.robotcore.external.Telemetry;



// Copyright (c) 2016-2019 Aedan Cullen and/or Evolution Robotics.


public class AutopilotHost {

    Telemetry telemetry;

    public enum NavigationStatus {RUNNING, STOPPED};

    private NavigationStatus navigationStatus = NavigationStatus.STOPPED;

    public int countsToStable;
    public double navigationUnitsToStable;
    public double orientationUnitsToStable;

    public double[] navigationTarget = new double[3];
    public double orientationTarget;
    public double navigationGain;
    public double orientationGain;
    public double navigationMax;
    public double navigationMin;
    public double orientationMax;
    public boolean useOrientation;
    public boolean useTranslation;
    public boolean fullStop;
    public boolean diffMode;

    PIDVelocityAdjuster chosenPowerAdjuster;

    private double[] robotAttitude = new double[3];

    private double[] robotPosition = new double[3];

    private boolean[] navigationTargetInverts = null;
    private boolean orientationTargetInvert = false;

    public AutopilotHost(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void setupVelocityPID(double Kp, double Ki, double Kd, double actualPeakRate) {
        chosenPowerAdjuster = new PIDVelocityAdjuster(Kp, Ki, Kd, actualPeakRate);
    }

    public void setCountsToStable(int countsToStable) {
        this.countsToStable = countsToStable;
    }

    public void setNavigationUnitsToStable(double navigationUnitsToStable) {
        this.navigationUnitsToStable = navigationUnitsToStable;
    }

    public void setOrientationUnitsToStable(double orientationUnitsToStable) {
        this.orientationUnitsToStable = orientationUnitsToStable;
    }

    public boolean getOrientationTargetInvert() {
        return this.orientationTargetInvert;
    }

    public void setOrientationTargetInvert(boolean orientationTargetInvert) {
        this.orientationTargetInvert = orientationTargetInvert;
    }

    public boolean[] getNavigationTargetInverts() {
        return this.navigationTargetInverts;
    }

    public void setNavigationTargetInverts(boolean[] navigationTargetInverts) {
        this.navigationTargetInverts = navigationTargetInverts;
    }

    public void telemetryUpdate() {
        telemetry.addData("* AutopilotHost", "\n" +
        "\t status:  " + navigationStatus.toString().toLowerCase() + "\n" +
        "\t target:  " + round(navigationTarget[0]) + ",  " + round(navigationTarget[1]) + ",  " + round(navigationTarget[2]) + "\n" +
        "\t position:  " + round(robotPosition[0]) + ",  " + round(robotPosition[1]) + ",  " + round(robotPosition[2]) + "\n" +
        "\t attitude:  " + round(robotAttitude[0]) + ",  " + round(robotAttitude[1]) + ",  " + round(robotAttitude[2]));
        //telemetry.update();
    }

    public void communicate(AutopilotTracker tracker) {
        tracker.update();

        robotAttitude = tracker.getRobotAttitude();
        robotPosition = tracker.getRobotPosition();
    }

    public NavigationStatus getNavigationStatus() {
        return navigationStatus;
    }

    public void setNavigationStatus(NavigationStatus navigationStatus) {
        this.navigationStatus = navigationStatus;

        lastDistanceToTarget = -1;
        distanceDecreased = false;
        if (chosenPowerAdjuster != null) {
            chosenPowerAdjuster.reset();
        }
    }

    public void setNavigationTarget(AutopilotSegment target) {
        setNavigationTarget(
                target.navigationTarget,
                target.orientationTarget,
                target.navigationGain,
                target.orientationGain,
                target.navigationMax,
                target.navigationMin,
                target.orientationMax,
                target.useOrientation,
                target.useTranslation,
                target.fullStop,
                target.diffMode
        );
    }

    private void applyOrientationTargetInvert() {
        this.orientationTarget = (Math.PI*2) - this.orientationTarget;
    }

    private void applyNavigationTargetInverts() {
        for (int i=0; i<3; i++){
            if (this.navigationTargetInverts[i]) {
                this.navigationTarget[i] = -this.navigationTarget[i];
            }
        }
    }

    public void setNavigationTarget(
            double[] navigationTarget,
            double orientationTarget,
            double navigationGain,
            double orientationGain,
            double navigationMax,
            double navigationMin,
            double orientationMax,
            boolean useOrientation,
            boolean useTranslation,
            boolean fullStop,
            boolean diffMode)
    {

        this.navigationTarget = navigationTarget;
        this.orientationTarget = orientationTarget;
        this.navigationGain = navigationGain;
        this.orientationGain = orientationGain;
        this.navigationMax = navigationMax;
        this.navigationMin = navigationMin;
        this.orientationMax = orientationMax;
        this.useOrientation = useOrientation;
        this.useTranslation = useTranslation;
        this.fullStop = fullStop;
        this.diffMode = diffMode;

        if (this.navigationTargetInverts != null) {
            this.applyNavigationTargetInverts();
        }

        if (this.orientationTargetInvert) {
            this.applyOrientationTargetInvert();
        }

        lastDistanceToTarget = -1;
        distanceDecreased = false;
        if (chosenPowerAdjuster != null) {
            chosenPowerAdjuster.reset();
        }
    }

    public double[] getNavigationTarget() {
        return navigationTarget;
    }

    public double[] getRobotAttitude() {
        return robotAttitude;
    }

    public double[] getRobotPosition() {
        return robotPosition;
    }

    private static boolean hasReached(double param1, double param2, double threshold) {
        return (Math.abs(param2 - param1) < threshold);
    }

    private static double round(double in) {
        double roundOff = (double) Math.round(in * 100) / 100;
        return roundOff;
    }

    double lastDistanceToTarget;
    boolean distanceDecreased;

    private void updateLasts(double distanceToTarget) {
        if (distanceToTarget < lastDistanceToTarget && lastDistanceToTarget != -1) {
            distanceDecreased = true;
        }
        lastDistanceToTarget = distanceToTarget;
    }

    private double[] applyDiff(double[] yxh) {
        double[] output = new double[3];
        output[0] = yxh[0];
        if (yxh[0] > 0) {
            output[2] = -yxh[1];
        }
        else {
            output[2] = yxh[1];
        }
        output[2] *= orientationGain * (Math.PI / 2);
        return output;
    }

    public double[] navigationTick() {
        return navigationTick(0);
    }

    public double[] navigationTick(double deltaPos) {
        if (diffMode) {
            fullStop = false;
        }


        if (navigationStatus == NavigationStatus.STOPPED) {
            return new double[3];
        }

        double hErr = Math.asin(Math.sin(orientationTarget - robotAttitude[0]));
        if (Math.cos(orientationTarget - robotAttitude[0]) < 0) {
            if (Math.sin(orientationTarget - robotAttitude[0]) > 0) {hErr = 2*Math.PI - hErr;} else {hErr = -2*Math.PI - hErr;}
        }

        double xErr = navigationTarget[0] - robotPosition[0];
        double yErr = navigationTarget[1] - robotPosition[1];

        double hCorr = Math.max(-orientationMax, Math.min(orientationMax, hErr * orientationGain));

        double translateTargAngle = -Math.atan(xErr / yErr);
        if (yErr < 0) {translateTargAngle += Math.PI;}

        //translateTargAngle = compensateDeviations(translateTargAngle);

        double finalAngle = translateTargAngle - robotAttitude[0];


        double distance = Math.sqrt(Math.pow(xErr, 2) + Math.pow(yErr, 2));
        double chosenPower = Math.max(navigationMin, Math.min(navigationMax, distance * navigationGain));

        if (chosenPowerAdjuster != null) {
            chosenPower = chosenPowerAdjuster.adjust(chosenPower, deltaPos);
            //Log.v("chosenPower", ""+ chosenPower);
        }

        double xCorr = chosenPower * -Math.sin(finalAngle);
        double yCorr = chosenPower * Math.cos(finalAngle);


        boolean boolReached = true;

        /*if (diffMode) {
            if (lastDistanceToTarget != -1) {
                boolReached = boolReached && (distanceDecreased && (distance > lastDistanceToTarget));
            }
            else {
                boolReached = false;
            }
        }*/
        if (useOrientation && !diffMode) {
            boolReached = boolReached && hasReached(hErr, 0, orientationUnitsToStable);
        }
        if (useTranslation && !diffMode) {
            boolReached = boolReached && hasReached(distance, 0, navigationUnitsToStable);
        }

        if (boolReached) {
            nTimesStable++;
        }
        else {
            nTimesStable = 0;
        }


        boolean rapidStopSatisfied = false;
        if (lastDistanceToTarget != -1) {
            rapidStopSatisfied = (distanceDecreased && (distance > lastDistanceToTarget));
        }

        if (!fullStop && rapidStopSatisfied) {
            navigationStatus = NavigationStatus.STOPPED;
        }
        if (fullStop && nTimesStable > countsToStable) {
            navigationStatus = NavigationStatus.STOPPED;
        }


        updateLasts(distance);

        if (hasReached(distance, 0, navigationUnitsToStable)) {
            xCorr = 0;
            yCorr = 0;
        }

        if (hasReached(hErr, 0, orientationUnitsToStable)) {
            hCorr = 0;
        }

        double[] ret = new double[3];

        if (useOrientation && !diffMode) {
            ret[2] = hCorr;
        }
        if (useTranslation) {
            ret[0] = yCorr;
            ret[1] = xCorr;
        }

        if (diffMode) {
            ret = applyDiff(ret);
        }

        return ret;

    }


    int nTimesStable;

}
