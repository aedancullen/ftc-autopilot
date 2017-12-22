package com.evolutionftc.autopilot;


// this is a snazzy autopilot for ftc made by aedan.
// it was made somewhat for team #9867, 'evolution'
// it uses lots of maths.
// copyright 2017 aedan cullen.


// MaxBotix MB1043 Sonar (raw X/Y) Variant of AutopilotTracker


import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class AutopilotTrackerMso extends AutopilotTracker {

	long msAtLastRxKick;

	private AnalogInput MbX;
	private AnalogInput MbY;

	private DigitalChannel MbRx; // ranging trigger pin
	
	double MbXOffset;
	double MbYOffset;

	// HRLV series with 3.3V supply on REV Robotics ADC, from experimental data gathered
	double inchesPerVolt = 73.123;

	// HRLV series - 10Hz refresh rate maximum using Rx pin trigger
	double rxKickIntervalMs = 100;
  
        private double[] robotPosition = new double[3];
	private double[] robotAttitude = new double[3];

	private double voltageToInches(double voltage){
		return voltage * inchesPerVolt;
	}
	

	public AutopilotTrackerMso(DigitalChannel MbRx, AnalogInput MbX, AnalogInput MbY, double MbXOffset, double MbYOffset) {

		this.MbRx = MbRx;
		MbRx.setMode(DigitalChannel.Mode.OUTPUT);
		this.MbX = MbX;
		this.MbY = MbY;
		this.MbXOffset = MbXOffset;
		this.MbYOffset = MbYOffset;
    	}


	public void update() {

		if (System.currentTimeMillis() > msAtLastRxKick + rxKickIntervalMs) {
			msAtLastRxKick = System.currentTimeMillis();
			MbRx.setState(true);
			MbRx.setState(false);
		}

		double distMbX = voltageToInches(MbX.getVoltage()) + MbXOffset;
		double distMbY = voltageToInches(MbY.getVoltage()) + MbYOffset;
		
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
