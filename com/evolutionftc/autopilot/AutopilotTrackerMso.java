package com.evolutionftc.autopilot;


// this is a snazzy autopilot for ftc made by aedan.
// it was made somewhat for team #9867, 'evolution'
// it uses lots of maths.
// copyright 2017 aedan cullen.


// MaxBotix MB1043 Sonar (raw X/Y) Variant of AutopilotTracker


import com.qualcomm.robotcore.hardware.AnalogInput;

public class AutopilotTrackerMso extends AutopilotTracker {

	private AnalogInput MbX;
	private AnalogInput MbY;
	
	double MbXOffset;
	double MbYOffset;
	
	double VperMM = (3.3) / (1024.0 * 5.0);
  
        private double[] robotPosition = new double[3];
	private double[] robotAttitude = new double[3];
	

	public AutopilotTrackerMso(AnalogInput MbX, AnalogInput MbY, double MbXOffset, double MbYOffset) {
        
		this.MbX = MbX;
		this.MbY = MbY;
		this.MbXOffset = MbXOffset;
		this.MbYOffset = MbYOffset;
    	}


	public void update() {

		double distMbX = (MbX.getVoltage() / VperMM + MbXOffset) / 10.0; // to cm
		double distMbY = (MbY.getVoltage() / VperMM + MbYOffset) / 10.0;
		
		robotPosition [0] = distMbX;
		robotPosition [1] = distMbY;
    	}

	public double[] getRobotPosition() {
		return robotPosition;
	}

	public double[] getRobotAttitude() {
		return robotAttitude;
	}

    public void setRobotPosition(double[] position) {
      robotPosition = position;
    }

	public void setRobotAttitude(double[] attitude) {
		robotAttitude = attitude;
	}

}
