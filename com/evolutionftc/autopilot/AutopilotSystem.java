package com.evolutionftc.autopilot;

import android.content.Context;

import com.qualcomm.robotcore.robocol.Telemetry;
import java.io.IOException;


// this is a snazzy autopilot for ftc made by aedan.
// it was made somewhat for team #9867, 'evolution'
// it uses lots of maths.
// it is gpl licensed.
// copyright 2017 aedan cullen.


public abstract class AutopilotSystem {

	private Telemetry telemetry;
	private Context appContext;
	private AutopilotTracker tracker;
	
	public AutopilotHost host;
	public AutopilotPath pathFollower;
	
	private AutopilotSegment currentSegment;

    public AutopilotSystem() {}
	
	public AutopilotSystem(AutopilotTracker tracker, Telemetry telemetry, Context appContext) {
		host = new AutopilotHost(telemetry, appContext);
		this.tracker = tracker;
		this.telemetry = telemetry;
		this.appContext = appContext;
	}
	
	public void beginPathTravel(String pathName) throws IOException {
		pathFollower = new AutopilotPath(pathName, telemetry);
	}
	
	public abstract void onSegmentTransition(AutopilotSegment previous, AutopilotSegment next, boolean wasOkayToContinue);
	
	public abstract boolean shouldContinue(AutopilotSegment segment,
                                           double[] robotAttitude,
                                           double[] robotAcceleration,
                                           double[] robotVelocity,
                                           double[] robotPosition);

    public double[] systemTick() {
        host.communicate(tracker);

        host.navigationTickDifferential();
		
        if (host.getNavigationStatus() == AutopilotHost.ProcessStatus.STOPPED) {
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
                                host.getRobotAcceleration(),
                                host.getRobotVelocity(),
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

    }

}
