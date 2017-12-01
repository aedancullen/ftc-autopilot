package com.evolutionftc.autopilot;


// this is a snazzy autopilot for ftc made by aedan.
// it was made somewhat for team #9867, 'evolution'
// it uses lots of maths.
// copyright 2017 aedan cullen.


// MaxBotix MB1043 Sonar (raw X/Y) Variant of AutopilotTracker



public class AutopilotTrackerMso {

	private AnalogInput MbX;
	private AnalogInput MbY;
	
	double MbXOffset;
	double MbYOffset;
  
        private double[] robotPosition = new double[3];
	private double[] robotAttitude = new double[3];
	

	public AutopilotTracker(AnalogInput MbX, AnalogInput MbY, double MbXOffset, double MbYOffset) {
        
		this.MbX = MbX;
		this.MbY = MbY;
		this.MbXOffset = MbXOffset;
		this.MbYOffset = MbYOffset;
    	}


	public void update() {

		double distMbX = 0 + MbXOffset;
		double distMbY = 0 + MbYOffset;
		
		robotPosition =  new double[2]{distMbX, distMbY};
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
