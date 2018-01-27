package com.evolutionftc.autopilot;


import android.content.Context;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.IOException;


// this is a snazzy autopilot for ftc made by aedan.
// it was made somewhat for team #9867, 'evolution'
// it uses lots of maths.
// copyright 2017 aedan cullen.


public class AutopilotSystem {

	private Telemetry telemetry;
    private Context appContext;
	public AutopilotTracker tracker;
	
	public AutopilotHost host;
	public AutopilotPath pathFollower;
	
	private AutopilotSegment currentSegment;

	public AutopilotSystem(){}
	
	public AutopilotSystem(AutopilotTracker tracker, Telemetry telemetry, Context appContext) {
		host = new AutopilotHost(telemetry);
		this.tracker = tracker;
		this.telemetry = telemetry;
        this.appContext = appContext;
	}
	
	public void beginPathTravel(String pathName) {
		pathFollower = new AutopilotPath(pathName, telemetry, appContext);

	}
	
	public void onSegmentTransition(AutopilotSegment previous, AutopilotSegment next, boolean wasOkayToContinue) {}
	
	public boolean shouldContinue(AutopilotSegment segment,
                                           double[] robotAttitude,
                                           double[] robotPosition) {
        return true;
    }

    public double[] systemTickDifferential() {
        host.communicate(tracker);

        host.telemetryUpdate();

        if (pathFollower == null) {
            telemetry.update();
            return new double[2];
        }

        pathFollower.telemetryUpdate();

        telemetry.update();

        double[] res = host.navigationTickDifferential();
		
        if (host.getNavigationStatus() == AutopilotHost.NavigationStatus.STOPPED) {
            AutopilotSegment newSegment = pathFollower.moveOnSuccess();
            onSegmentTransition(currentSegment, newSegment, true);
            currentSegment = newSegment;
            if (currentSegment != null) {
                host.setNavigationTarget(currentSegment);
                host.setNavigationStatus(AutopilotHost.NavigationStatus.RUNNING);
                host.communicate(tracker);
                return host.navigationTickDifferential();
            }
            else {
                return new double[2];
            }
        }
        else if (shouldContinue(currentSegment,
                                host.getRobotAttitude(),
                                host.getRobotPosition()) == false)
        {
            AutopilotSegment newSegment = pathFollower.moveOnFailure();
            onSegmentTransition(currentSegment, newSegment, false);
            currentSegment = newSegment;
            if (currentSegment != null) {
                host.setNavigationTarget(currentSegment);
                host.setNavigationStatus(AutopilotHost.NavigationStatus.RUNNING);
                host.communicate(tracker);
                return host.navigationTickDifferential();
            }
            else {
                return new double[2];
            }
        }
        else {
            return res;
        }

    }

    public double[] systemTickRaw() {
        host.communicate(tracker);

        host.telemetryUpdate();

        if (pathFollower == null) {
            telemetry.update();
            return new double[2];
        }

        pathFollower.telemetryUpdate();

        telemetry.update();

        double[] res = host.navigationTickRaw();

        if (host.getNavigationStatus() == AutopilotHost.NavigationStatus.STOPPED) {
            AutopilotSegment newSegment = pathFollower.moveOnSuccess();
            onSegmentTransition(currentSegment, newSegment, true);
            currentSegment = newSegment;
            if (currentSegment != null) {
                host.setNavigationTarget(currentSegment);
                host.setNavigationStatus(AutopilotHost.NavigationStatus.RUNNING);
                host.communicate(tracker);
                return host.navigationTickRaw();
            }
            else {
                return new double[2];
            }
        }
        else if (shouldContinue(currentSegment,
                host.getRobotAttitude(),
                host.getRobotPosition()) == false)
        {
            AutopilotSegment newSegment = pathFollower.moveOnFailure();
            onSegmentTransition(currentSegment, newSegment, false);
            currentSegment = newSegment;
            if (currentSegment != null) {
                host.setNavigationTarget(currentSegment);
                host.setNavigationStatus(AutopilotHost.NavigationStatus.RUNNING);
                host.communicate(tracker);
                return host.navigationTickRaw();
            }
            else {
                return new double[2];
            }
        }
        else {
            return res;
        }

    }

}
