package com.evolutionftc.autopilot;

import android.content.Context;

import com.qualcomm.robotcore.robocol.Telemetry;
import java.io.IOException;


public abstract class AutopilotSystem {

	private Telemetry telemetry;
	private Context appContext;
	private AutopilotTracker tracker;
	
	public AutopilotHost host;
	public PathFollower pathFollower;
	
	private PathSegment currentSegment;

    public AutopilotSystem() {}
	
	public AutopilotSystem(AutopilotTracker tracker, Telemetry telemetry, Context appContext) {
		host = new AutopilotHost(telemetry, appContext);
		this.tracker = tracker;
		this.telemetry = telemetry;
		this.appContext = appContext;
	}
	
	public void beginPathTravel(String pathName) throws IOException {
		pathFollower = new PathFollower(pathName, telemetry);
	}
	
	public abstract void onSegmentTransition(PathSegment previous, PathSegment next, boolean wasOkayToContinue);
	
	public abstract boolean shouldContinue(PathSegment segment,
                                           double[] robotAttitude,
                                           double[] robotAcceleration,
                                           double[] robotVelocity,
                                           double[] robotPosition);

    public double[] systemTick() {
        host.communicate(tracker);
		
        if (host.getNavigationStatus() == AutopilotHost.ProcessStatus.STOPPED) {
            PathSegment newSegment = pathFollower.moveOnSuccess();
            onSegmentTransition(currentSegment, newSegment, true);
            currentSegment = newSegment;
            if (currentSegment != null) {
                host.setNavigationTarget(currentSegment);
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
            PathSegment newSegment = pathFollower.moveOnFailure();
            onSegmentTransition(currentSegment, newSegment, false);
            currentSegment = newSegment;
            if (currentSegment != null) {
                host.setNavigationTarget(currentSegment);
            }
            else {
                return new double[2];
            }
        }
        return host.navigationTickDifferential();
    }

}
