package com.evolutionftc.autopilot;


import android.content.Context;
import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;


// Copyright (c) 2016-2019 Aedan Cullen and/or Evolution Robotics.


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

		Log.v("AutopilotVisBcast", status+","+robotX+","+robotY+","+robotH);
    }


    public double[] systemTick() {
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

        double[] res = host.navigationTick();

        if (host.getNavigationStatus() == AutopilotHost.NavigationStatus.STOPPED) {
            AutopilotSegment newSegment = pathFollower.moveOnSuccess();
            onSegmentTransition(currentSegment, newSegment, true);
            currentSegment = newSegment;
            if (currentSegment != null) {
                host.setNavigationTarget(currentSegment);
                host.setNavigationStatus(AutopilotHost.NavigationStatus.RUNNING);
                host.communicate(tracker);
                return host.navigationTick();
            }
            else {
                return new double[3];
            }
        }
        else if (shouldContinue(currentSegment,
                host.getRobotAttitude(),
                host.getRobotPosition()) == false)
        {
            while (shouldContinue(currentSegment,
                    host.getRobotAttitude(),
                    host.getRobotPosition()) == false) {
                AutopilotSegment newSegment = pathFollower.moveOnFailure();
                onSegmentTransition(currentSegment, newSegment, false);
                currentSegment = newSegment;
                if (currentSegment == null) {
                    return new double[3];
                }
                host.setNavigationTarget(currentSegment);
                host.setNavigationStatus(AutopilotHost.NavigationStatus.RUNNING);
                host.communicate(tracker);
            }
            return host.navigationTick();
        }
        else {
            return res;
        }

    }

}
