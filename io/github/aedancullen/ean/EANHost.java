package io.github.cn04.eanhost;

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


public class EANHost {

    Telemetry telemetry;
    Context appContext;

    public enum ProcessStatus {RUNNING, STOPPED};

    private ProcessStatus navigationStatus = ProcessStatus.STOPPED;

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

    public EANHost(double[] initialPosition, Telemetry telemetry, Context appContext) {
        this.telemetry = telemetry;
        this.appContext = appContext;
        robotPosition = initialPosition;
    }

    private void telemetryUpdate() {
        telemetry.addData("* EANHost", "\n" +
                "\t nav: " + navigationStatus.toString().toLowerCase() + "\n" +
                "\t trg: " + navigationTarget[0] + "," + navigationTarget[1] + "," + navigationTarget[2] + "\n" +
                "\t pos: " + robotPosition[0] + "," + robotPosition[1] + "," + robotPosition[2]);
    }
	
    public void communicate(EANTracker tracker, boolean globalRestartOnConnect) {
    
    }

    public ProcessStatus getNavigationStatus() {
        return navigationStatus;
    }

    public void setNavigationTarget(PathSegment target) {
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

    public double[] navigationTickDifferential() {
		telemetryUpdate();
		
        if (
                    hasReached(robotPosition[0], navigationTarget[0], accuracyThreshold[0]) &&
                    hasReached(robotPosition[1], navigationTarget[1], accuracyThreshold[1]) &&
                    hasReached(robotPosition[2], navigationTarget[2], accuracyThreshold[2])
           )
        {
            navigationStatus = ProcessStatus.STOPPED;
            return null;
        }
        else {
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
            double angle = Math.toDegrees(Math.atan(distY / distX));
            double powerLeft = Math.max((basePower - powerAdj), lowestPower) + (angle * steeringGain);
            double powerRight = Math.max((basePower - powerAdj), lowestPower) - (angle * steeringGain);
            return new double[] {powerLeft, powerRight};
        }
    }

}
