package com.evolutionftc.autopilot;


// this is a snazzy autopilot for ftc made by aedan.
// it was made somewhat for team #9867, 'evolution'
// it uses lots of maths.
// it is gpl licensed.
// copyright 2017 aedan cullen.


public class AutopilotTracker {

	private DcMotor left;
	private DcMotor right;

	private IMUTHINGY imu;

	private double[] robotAttitude = new double[3];

    private double[] robotAcceleration = new double[3];
    private double[] robotVelocity = new double[3];
    private double[] robotPosition = new double[3];


    private static double[][] matmul(double[][] firstarray, double[][] secondarray) {

        double [][] result = new double[firstarray.length][secondarray[0].length];

		/* Loop through each and get product, then sum up and store the value */
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

    	// See http://planning.cs.uiuc.edu/node104.html

    	double[][] out = new double[4][4];

    	double x = xyz[0];
  		double y = xyz[1];
  		double z = xyz[2];
  		double h = hpr[0];
  		double p = hpr[1];
  		double r = hpr[2];


  		out[0][0] = Math.cos(h)*Math.cos(p);
  		out[0][1] = (Math.cos(h)*Math.sin(p)*Math.sin(r)) - (Math.sin(h)*Math.cos(r));
  		out[0][2] = (Math.cos(h)*Math.sin(p)*Math.cos(r)) + (Math.sin(h)*Math.sin(r));
  		out[0][3] = x;
  		out[1][0] = Math.sin(h)*Math.cos(p);
  		out[1][1] = (Math.sin(h)*Math.sin(p)*Math.sin(r)) + (Math.cos(h)*Math.cos(r));
  		out[1][2] = (Math.sin(h)*Math.sin(p)*Math.cos(r)) - (Math.cos(h)*Math.sin(r));
  		out[1][3] = y;
  		out[2][0] = -Math.sin(p);
  		out[2][1] = Math.cos(p)*Math.sin(r);
  		out[2][2] = Math.cos(p)*Math.cos(r);
  		out[2][3] = z;
  		out[3][0] = 0.0;
  		out[3][1] = 0.0;
  		out[3][2] = 0.0;
  		out[3][3] = 1.0;

  		return out;
    }

    private static double[] transform(double[] point, double[] translation, double[] rotation) {

    	// Transform a 3D body by rotation and then translation. See http://planning.cs.uiuc.edu/node104.html

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


	public AutopilotTracker(DcMotor left, DcMotor right, IMUTHINGY imu) {
        this.left = left;
        this.right = right;
        this.imu = imu;
    }


	public double[] getRobotAttitude() {
        return imu.GETHPRSOMEHOW()
    }

    public double[] getRobotAcceleration() {
        // not todai, boi
    }

    public double[] getRobotVelocity() {
    	// not todai, boi
    }

    public double[] getRobotPosition() {

    	double yval = (right.ENCODERSOMEHOW() + left.ENCODERSOMEHOW()) / 2.0;

    	// reset encoder somehow??

    	double[] translation = {0.0, yval, 0.0};

    	// isn't this so noice and clean
    	robotPosition = transform(robotPosition, translation, imu.GETHPRSOMEHOW());

    	return robotPosition;
    }

}
