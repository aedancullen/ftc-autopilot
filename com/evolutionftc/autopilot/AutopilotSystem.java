package com.evolutionftc.autopilot;


import android.content.Context;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.IOException;


// Copyright (c) 2016-2018 Aedan Cullen and/or Evolution Robotics.


public class AutopilotSystem {

	private static final int VISUALIZER_BROADCAST_INTERVAL_MS = 50;

	private long msAtLastBroadcast;

	private Telemetry telemetry;
	private Context appContext;
	public AutopilotTracker tracker;

	public AutopilotHost host;
	public AutopilotPath pathFollower;

	private AutopilotSegment currentSegment;

	public AutopilotSystem(){}

	private boolean visualizerBroadcastEnabled;

	public AutopilotSystem(AutopilotTracker tracker, Telemetry telemetry, Context appContext, boolean visualizerBroadcastEnabled) {
		host = new AutopilotHost(telemetry);
		this.tracker = tracker;
		this.telemetry = telemetry;
		this.appContext = appContext;
		this.visualizerBroadcastEnabled = visualizerBroadcastEnabled;
		this.msAtLastBroadcast = System.currentTimeMillis();
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

    private void doVisualizerBroadcast(AutopilotHost broadcastHost) {
		String status = broadcastHost.getNavigationStatus().toString().toLowerCase();
		double robotX = broadcastHost.getRobotPosition()[0];
		double robotY = broadcastHost.getRobotPosition()[1];
		double robotH = broadcastHost.getRobotAttitude()[0];

		Log.v("AutopilotVisualizerBroadcast", status+","+robotX+","+robotY+","+robotH);
    }

    public double[] systemTickDifferential() {
		long timeNow = System.currentTimeMillis();
		if (visualizerBroadcastEnabled &&
	    	timeNow - msAtLastBroadcast > VISUALIZER_BROADCAST_INTERVAL_MS)
		{
			this.doVisualizerBroadcast(host);
			msAtLastBroadcast = timeNow;
		}

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

	public double[] systemTickFoursides() {
		long timeNow = System.currentTimeMillis();
		if (visualizerBroadcastEnabled &&
	    	timeNow - msAtLastBroadcast > VISUALIZER_BROADCAST_INTERVAL_MS)
		{
			this.doVisualizerBroadcast(host);
			msAtLastBroadcast = timeNow;
		}

        host.communicate(tracker);

        host.telemetryUpdate();

        if (pathFollower == null) {
            telemetry.update();
            return new double[3];
        }

        pathFollower.telemetryUpdate();

        telemetry.update();

        double[] res = host.navigationTickFoursidesl();

        if (host.getNavigationStatus() == AutopilotHost.NavigationStatus.STOPPED) {
            AutopilotSegment newSegment = pathFollower.moveOnSuccess();
            onSegmentTransition(currentSegment, newSegment, true);
            currentSegment = newSegment;
            if (currentSegment != null) {
                host.setNavigationTarget(currentSegment);
                host.setNavigationStatus(AutopilotHost.NavigationStatus.RUNNING);
                host.communicate(tracker);
                return host.navigationTickFoursides();
            }
            else {
                return new double[3];
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
                return host.navigationTickFoursides();
            }
            else {
                return new double[3];
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
