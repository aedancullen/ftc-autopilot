package com.evolutionftc.autopilot;

import android.app.PendingIntent;
import android.content.Context;
import android.content.Intent;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;

import com.qualcomm.robotcore.robocol.Telemetry;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;


public class AutopilotHost {

    Telemetry telemetry;
    Context appContext;

    public enum NavigationStatus {RUNNING, STOPPED};

    private NavigationStatus navigationStatus = NavigationStatus.STOPPED;

    private double basePower;
    private double lowestPower;
    private double powerGain;

    private double navigationHalfway;
    private boolean rampUp;
    private boolean rampDown;

    private double[] navigationTarget = new double[3];
    private double steeringGain;
    private double[] accuracyThreshold;

    private double[] robotAttitude = new double[3];

    private double[] robotAcceleration = new double[3];
    private double[] robotVelocity = new double[3];
    private double[] robotPosition = new double[3];

    public AutopilotHost(Telemetry telemetry, Context appContext) {
        this.telemetry = telemetry;
        this.appContext = appContext;
	telemetryUpdate();
    }

    private void telemetryUpdate() {
        telemetry.addData("* AutopilotHost", "\n" +
                "\t nav: " + navigationStatus.toString().toLowerCase() + "\n" +
                "\t trg: " + round(navigationTarget[0]) + "," + round(navigationTarget[1]) + "," + round(navigationTarget[2]) + "\n" +
                "\t pos: " + round(robotPosition[0]) + "," + round(robotPosition[1]) + "," + round(robotPosition[2]) + "\n" +
                "\t att: " + round(robotAttitude[0]) + "," + round(robotAttitude[1]) + "," + round(robotAttitude[2]);
    }
	
    public void communicate(AutopilotTracker tracker) {
    	telemetryUpdate();
    }

    public ProcessStatus getNavigationStatus() {
        return navigationStatus;
    }
	
    public void setNavigationStatus(NavigationStatus navigationStatus) {
	this.navigationStatus = navigationStatus;
	telemetryUpdate();
    }

    public void setNavigationTarget(AutopilotSegment target) {
        setNavigationTarget(target.navigationTarget, target.steeringGain, target.accuracyThreshold, target.basePower, target.lowestPower, target.powerGain, target.rampUp, target.rampDown);
    }

    public void setNavigationTarget(double[] navigationTarget, double steeringGain, double[] accuracyThreshold, double basePower, double lowestPower, double powerGain, boolean rampUp, boolean rampDown) {
        this.navigationTarget = navigationTarget;
        this.steeringGain = steeringGain;
        this.accuracyThreshold = accuracyThreshold;
        this.basePower = basePower;
        this.lowestPower = lowestPower;
        this.powerGain = powerGain;
        this.rampUp = rampUp;
        this.rampDown = rampDown;
        double distX = navigationTarget[0] - robotPosition[0];
        double distY = navigationTarget[1] - robotPosition[1];
        double dist = Math.sqrt(Math.pow(distX, 2) + Math.pow(distY, 2));
        navigationHalfway = dist / 2;
	telemetryUpdate();
    }

    public double[] getNavigationTarget() {
        return navigationTarget;
    }

    public double[] getRobotAttitude() {
        return robotAttitude;
    }

    public double[] getRobotAcceleration() {
        return robotAcceleration;
    }

    public double[] getRobotVelocity() {
        return robotVelocity;
    }

    public double[] getRobotPosition() {
        return robotPosition;
    }
	
    private static boolean hasReached(double param1, double param2, double threshold) {
        return (Math.abs(param2 - param1) < threshold);
    }

    private static double round(double in) {
        double roundOff = (double) Math.round(in * 100) / 100;
        return roundOff
    }

    public double[] navigationTickDifferential() {
		
        if (
                    hasReached(robotPosition[0], navigationTarget[0], accuracyThreshold[0]) &&
                    hasReached(robotPosition[1], navigationTarget[1], accuracyThreshold[1]) &&
                    hasReached(robotPosition[2], navigationTarget[2], accuracyThreshold[2])
           )
        {
            navigationStatus = NavigationStatus.STOPPED;
            return null;
        }
        else if (navigationStatus == NavigationStatus.RUNNING) {
            double distX = navigationTarget[0] - robotPosition[0];
            double distY = navigationTarget[1] - robotPosition[1];
            double dist = Math.sqrt(Math.pow(distX, 2) + Math.pow(distY, 2));
            double powerAdj = (dist - navigationHalfway) * powerGain;
            if (powerAdj > 0 && !rampUp) {
                powerAdj = 0;
            }
            if (powerAdj < 0 && !rampDown) {
                powerAdj = 0;
            }
            else if (powerAdj < 0 && rampDown) {
                powerAdj *= -1;
            }
            double angle = Math.atan(distY / distX) - robotAttitude[3];
            double powerLeft = Math.max((basePower - powerAdj), lowestPower) - (angle * steeringGain);
            double powerRight = Math.max((basePower - powerAdj), lowestPower) + (angle * steeringGain);
            return new double[] {powerLeft, powerRight};
        }

        telemetryUpdate();

    }

}
