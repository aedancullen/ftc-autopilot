

package com.evolutionftc.autopilot;



import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.robotcore.external.Telemetry;



// Copyright (c) 2016-2019 Aedan Cullen and/or Evolution Robotics.


public class AutopilotHost {

    Telemetry telemetry;

    public enum NavigationStatus {RUNNING, STOPPED};

    private NavigationStatus navigationStatus = NavigationStatus.STOPPED;

    public double countsToStable;
    public double navigationUnitsToStable;
    public double orientationUnitsToStable;

    public double[] navigationTarget;
    public double orientationTarget;
    public double navigationGain;
    public double orientationGain;
    public double navigationMax;
    public double navigationMin;
    public double orientationMax;
    public boolean useOrientation;

    private double[] robotAttitude = new double[3];

    private double[] robotPosition = new double[3];

    private boolean[] navigationTargetInverts = null;
    private boolean orientationTargetInvert = false;

    public AutopilotHost(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void setCountsToStable(double countsToStable) {
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
        tracker.setRobotAttitude(robotAttitude);
        tracker.setRobotPosition(robotPosition);
        tracker.update();

        robotAttitude = tracker.getRobotAttitude();
        robotPosition = tracker.getRobotPosition();
    }

    public NavigationStatus getNavigationStatus() {
        return navigationStatus;
    }

    public void setNavigationStatus(NavigationStatus navigationStatus) {
        this.navigationStatus = navigationStatus;

    }

    public void setNavigationTarget(AutopilotSegment target) {
        setNavigationTarget(target.navigationTarget, target.orientationTarget, target.navigationGain, target.orientationGain, target.navigationMax, target.navigationMin, target.orientationMax, target.useOrientation);
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

    public void setNavigationTarget(double[] navigationTarget, double orientationTarget, double navigationGain, double orientationGain, double navigationMax, double navigationMin, double orientationMax, boolean useOrientation) {
        this.navigationTarget = navigationTarget;
        this.orientationTarget = orientationTarget;
        this.navigationGain = navigationGain;
        this.orientationGain = orientationGain;
        this.navigationMax = navigationMax;
        this.navigationMin = navigationMin;
        this.orientationMax = orientationMax;
        this.useOrientation = useOrientation;

        if (this.navigationTargetInverts != null) {
            this.applyNavigationTargetInverts();
        }

        if (this.orientationTargetInvert) {
            this.applyOrientationTargetInvert();
        }

        lastRobotPosition = null;

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

    public void setRobotPosition(double[] position) {
        robotPosition = position;

        lastRobotPosition = null;
    }

    private static boolean hasReached(double param1, double param2, double threshold) {
        return (Math.abs(param2 - param1) < threshold);
    }

    private static double round(double in) {
        double roundOff = (double) Math.round(in * 100) / 100;
        return roundOff;
    }


    double lastTranslateTargAngle;
    double[] lastRobotPosition;

    private void updateLasts(double translateTargAngle) {
        lastTranslateTargAngle = translateTargAngle;
        lastRobotPosition = robotPosition;
    }


    private double compensateDeviations(double plainTargAngle) {
        if (lastRobotPosition == null) {return plainTargAngle;}

        double movedX = robotPosition[0] - lastRobotPosition[0];
        double movedY = robotPosition[1] - lastRobotPosition[1];
        double movedAngle = -Math.tan(movedX / movedY);
        if (movedY < 0) {movedAngle += Math.PI;}

        double deviatedAngle = movedAngle - lastTranslateTargAngle;
        return plainTargAngle - deviatedAngle;
    }


    public double[] navigationTick() {

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

        translateTargAngle = compensateDeviations(translateTargAngle);

        double finalAngle = translateTargAngle - robotAttitude[0];


        double distance = Math.sqrt(Math.pow(xErr, 2) + Math.pow(yErr, 2));
        double chosenPower = Math.max(navigationMin, Math.min(navigationMax, distance * navigationGain));

        double xCorr = chosenPower * -Math.sin(finalAngle);
        double yCorr = chosenPower * Math.cos(finalAngle);

        if (
                hasReached(robotPosition[0], navigationTarget[0], navigationUnitsToStable)
                && hasReached(robotPosition[1], navigationTarget[1], navigationUnitsToStable)
                && hasReached(robotAttitude[0], orientationTarget, orientationUnitsToStable)
        ) {
            nTimesStable++;
        }
        else {
            nTimesStable = 0;
        }



        if (nTimesStable > countsToStable) {
            navigationStatus = NavigationStatus.STOPPED;
        }


        updateLasts(translateTargAngle);

        if (useOrientation) {
            return new double[] {yCorr, xCorr, hCorr};
        }
        else {
            return new double[] {yCorr, xCorr, 0};
        }

    }


    int nTimesStable;

}
