package com.evolutionftc.autopilot;


public class AutopilotTracker {

	private DcMotor left;
	private DcMotor right;

	private double[] robotAttitude = new double[3];

    private double[] robotAcceleration = new double[3];
    private double[] robotVelocity = new double[3];
    private double[] robotPosition = new double[3];


	public AutopilotTracker(DcMotor left, DcMotor right) {
        this.left = left;
        this.right = right;
    }


	public double[] getRobotAttitude() {
        
    }

    public double[] getRobotAcceleration() {
        
    }

    public double[] getRobotVelocity() {

    }

    public double[] getRobotPosition() {

    }

}
