package io.github.cn04.eanhost;

import android.content.Context;

import com.qualcomm.robotcore.robocol.Telemetry;
import java.io.IOException;


public abstract class EANSystem {

	private Telemetry telemetry;
	private Context appContext;
	private EANTracker tracker;
	
	public EANHost host;
	public PathFollower pathFollower;
	
	private PathSegment currentSegment;

    public EANSystem() {}
	
	public EANSystem(EANTracker tracker, Telemetry telemetry, Context appContext) {
		this.tracker = tracker;
		this.telemetry = telemetry;
		this.appContext = appContext;
	}
	
	public void beginPathTravel(double[] initialPosition, String pathName) throws IOException {
		host = new EANHost(initialPosition, telemetry, appContext);
		pathFollower = new PathFollower(pathName, telemetry);
                host.communicate(tracker);
	}
	
	public abstract void onSegmentTransition(PathSegment previous, PathSegment next, boolean wasOkayToContinue);
	
	public abstract boolean shouldContinue(PathSegment segment,
                                           double[] robotAttitude,
                                           double[] robotAcceleration,
                                           double[] robotVelocity,
                                           double[] robotPosition);

    public double[] systemTick() {
        host.communicate(tracker);
		
        if (host.getNavigationStatus() == EANHost.ProcessStatus.STOPPED) {
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
