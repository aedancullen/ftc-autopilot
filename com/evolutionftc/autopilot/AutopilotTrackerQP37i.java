package com.evolutionftc.autopilot;


// Copyright (c) 2016-2019 Aedan Cullen and/or Evolution Robotics.


// QP37i tracking driver


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class AutopilotTrackerQP37i extends AutopilotTracker {

	private DcMotor x;
	private DcMotor y;

	public boolean invertX;
	public boolean invertY;

	long xenc;
	long yenc;
	double ticksPerUnit;

	public BNO055IMU imu;

	int nSubsteps;
	
	private double[] rao = new double[3];

	private double[] robotPosition = new double[3];
	private double[] robotAttitude = new double[3];

	private double[] sensorPosRelativeToRobot = new double[3];
	private double[] negSensorPosRelativeToRobot = new double[3];


	private static double[][] matmul(double[][] firstarray, double[][] secondarray) {

		double [][] result = new double[firstarray.length][secondarray[0].length];

		for (int i = 0; i < firstarray.length; i++) {
			for (int j = 0; j < secondarray[0].length; j++) {
				for (int k = 0; k < firstarray[0].length; k++) {
					result[i][j] += firstarray[i][k] * secondarray[k][j];
				}
			}
		}

		return result;
	}

	private static double[][] buildTransform(double[] xyz, double[] hpr){

		double[][] out = new double[4][4];

		double x = xyz[0];
		double y = xyz[1];
		double z = xyz[2];
		double h = hpr[0];
		double p = hpr[1];
		double r = hpr[2];

                double sinh = Math.sin(h);
                double cosh = Math.cos(h);
                double sinp = Math.sin(p);
                double cosp = Math.cos(p);
                double sinr = Math.sin(r);
                double cosr = Math.cos(r);

		out[0][0] = cosh*cosp;
		out[0][1] = (cosh*sinp*sinr) - (sinh*cosr);
		out[0][2] = (cosh*sinp*cosr) + (sinh*sinr);
		out[0][3] = x;
		out[1][0] = sinh*cosp;
		out[1][1] = (sinh*sinp*sinr) + (cosh*cosr);
		out[1][2] = (sinh*sinp*cosr) - (cosh*sinr);
		out[1][3] = y;
		out[2][0] = -sinp;
		out[2][1] = cosp*sinr;
		out[2][2] = cosp*cosr;
		out[2][3] = z;
		out[3][0] = 0.0;
		out[3][1] = 0.0;
		out[3][2] = 0.0;
		out[3][3] = 1.0;

		return out;
	}

	private static double[] transform(double[] point, double[] translation, double[] rotation) {

		// Transform a 3D body by rotation and then translation.

		double[][] transform = buildTransform(point, rotation);

		double[][] params = {
			{translation[0]},
			{translation[1]},
			{translation[2]},
			{1.0}
		};

		double[][] result = matmul(transform, params);

		double[] out = new double[3];

		out[0] = result[0][0];
		out[1] = result[1][0];
		out[2] = result[2][0];

		return out;
	}


	public AutopilotTrackerQP37i(DcMotor x, DcMotor y, double[] sensorPosRelativeToRobot, double ticksPerUnit, BNO055IMU imu, int nSubsteps) {
		this.x = x;
		this.y = y;
		this.sensorPosRelativeToRobot = sensorPosRelativeToRobot;
		for (int i = 0; i < 3; i++) {
			negSensorPosRelativeToRobot[i] = -sensorPosRelativeToRobot[i];
		}
		this.imu = imu;
		this.ticksPerUnit = ticksPerUnit;
		this.nSubsteps = nSubsteps;

		//BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		//parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
		//parameters.calibrationDataFile = "BNO055IMUCalibration.json";

		//imu.initialize(parameters);
	}

	public void setInverts(boolean invertX, boolean invertY) {
		this.invertX = invertX;
		this.invertY = invertY;
	}


	public void update() {

		double[] oldRobotAttitude = new double[3];
		oldRobotAttitude[0] = robotAttitude[0];
		oldRobotAttitude[1] = robotAttitude[1];
		oldRobotAttitude[2] = robotAttitude[2];

		Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
		robotAttitude[0] = angles.firstAngle;
		robotAttitude[1] = 0;
		robotAttitude[2] = 0;
		for (int i = 0; i < 3; i++) {
			robotAttitude[i] -= rao[i];
		}

                // To sensor pos
                robotPosition = transform(robotPosition, sensorPosRelativeToRobot, robotAttitude);

		long ticksX = x.getCurrentPosition();
		long ticksY = y.getCurrentPosition();

		double xval = ((double)(ticksX - xenc) / ticksPerUnit);
		double yval = ((double)(ticksY - yenc) / ticksPerUnit);

		xenc = ticksX;
		yenc = ticksY;

		if (invertX) {xval = -xval;}
		if (invertY) {yval = -yval;}

		double[] translationDeltaPerStep = {xval / (double)nSubsteps, yval / (double)nSubsteps, 0.0};
		double[] rotationDeltaPerStep = new double[3];
		for (int i=0; i<3; i++) {
			// rotationDeltaPerStep * nSubsteps equals the difference between oldRobotAttitude and robotAttitude.
			rotationDeltaPerStep[i] = (robotAttitude[i] - oldRobotAttitude[i]) / (double)nSubsteps;
		}

		for (int i=0; i < nSubsteps; i++) {
			double[] robotAttitudeThisStep = new double[3];

			for (int j=0; j<3; j++) {
				// Add (i) mini-steps to the old starting attitude.
				robotAttitudeThisStep[j] = oldRobotAttitude[j] + (rotationDeltaPerStep[j] * i);
			}
			robotPosition = transform(robotPosition, translationDeltaPerStep, robotAttitudeThisStep);
		}

		// Back to actual robot pos
		robotPosition = transform(robotPosition, negSensorPosRelativeToRobot, robotAttitude);
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
		for (int i = 0; i < 3; i++) {
			rao[i] = robotAttitude[i] - attitude[i];
		}
	}

}
