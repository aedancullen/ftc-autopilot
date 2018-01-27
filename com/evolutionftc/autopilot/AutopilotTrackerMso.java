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

	long lastTimeSaneX = -1;
	long lastTimeSaneY = -1;

	int INSANITY_TIMEOUT = 1000; // max. number of ms of insane readings before we give in

	double SANE_INCHES_PER_TICK = 15; // generous, but good enough for filtering outliers

	// HRLV series with 3.3V supply on REV Robotics ADC, from experimental data gathered
	double inchesPerVolt = 73.123;

	// HRLV series - 10Hz refresh rate maximum using Rx pin trigger
	double rxKickIntervalMs = 100;
  
        private double[] robotPosition = new double[3];
	private double[] robotAttitude = new double[3];

	private double voltageToInches(double voltage){
		return voltage * inchesPerVolt;
	}

	private boolean sanityCheck(double lastInput, double currentInput) {
		if (Math.abs(currentInput - lastInput) > SANE_INCHES_PER_TICK) {
			return false;
		}
		else {
			return true;
		}
	}
	

	public AutopilotTrackerMso(DigitalChannel MbRx, AnalogInput MbX, AnalogInput MbY, double MbXOffset, double MbYOffset) {

		this.MbRx = MbRx;
		MbRx.setMode(DigitalChannel.Mode.OUTPUT);
		MbRx.setState(false);
		this.MbX = MbX;
		this.MbY = MbY;
		this.MbXOffset = MbXOffset;
		this.MbYOffset = MbYOffset;
    	}


	public void update() {

		//if (System.currentTimeMillis() > msAtLastRxKick + rxKickIntervalMs) {
		//	msAtLastRxKick = System.currentTimeMillis();
			MbRx.setState(true);
			MbRx.setState(false);
		//}

		double distMbX = voltageToInches(MbX.getVoltage()) + MbXOffset;
		double distMbY = voltageToInches(MbY.getVoltage()) + MbYOffset;

		long time = System.currentTimeMillis();

		if (sanityCheck(robotPosition[0], distMbX) || robotPosition[0] == 0 || time > lastTimeSaneX + INSANITY_TIMEOUT) {
			robotPosition[0] = distMbX;
			lastTimeSaneX = time;
		}
		if (sanityCheck(robotPosition[1], distMbY) || robotPosition[1] == 0 || time > lastTimeSaneY + INSANITY_TIMEOUT) {
			robotPosition[1] = distMbY;
			lastTimeSaneY = time;
		}
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
