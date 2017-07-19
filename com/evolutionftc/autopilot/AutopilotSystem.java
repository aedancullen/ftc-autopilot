package com.evolutionftc.autopilot;


import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.IOException;


// this is a snazzy autopilot for ftc made by aedan.
// it was made somewhat for team #9867, 'evolution'
// it uses lots of maths.
// it is gpl licensed.
// copyright 2017 aedan cullen.


public abstract class AutopilotSystem {

	private Telemetry telemetry;
	private AutopilotTracker tracker;
	
	public AutopilotHost host;
	public AutopilotPath pathFollower;
	
	private AutopilotSegment currentSegment;

    public AutopilotSystem() {}
	
	public AutopilotSystem(AutopilotTracker tracker, Telemetry telemetry) {
		host = new AutopilotHost(telemetry);
		this.tracker = tracker;
		this.telemetry = telemetry;
	}
	
	public void beginPathTravel(String pathName) throws IOException {
		pathFollower = new AutopilotPath(pathName, telemetry);
	}
	
	public abstract void onSegmentTransition(AutopilotSegment previous, AutopilotSegment next, boolean wasOkayToContinue);
	
	public abstract boolean shouldContinue(AutopilotSegment segment,
                                           double[] robotAttitude,
                                           double[] robotPosition);

    public double[] systemTick() {
        host.communicate(tracker);

        double[] res = host.navigationTickDifferential();
		
        if (host.getNavigationStatus() == AutopilotHost.NavigationStatus.STOPPED) {
            AutopilotSegment newSegment = pathFollower.moveOnSuccess();
            onSegmentTransition(currentSegment, newSegment, true);
            currentSegment = newSegment;
            if (currentSegment != null) {
                host.setNavigationTarget(currentSegment);
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

}
