package com.evolutionftc.autopilot;


// Quad-odometer tracker, not using IMU at all
// Full turn-corroboration version ("x" encoder mounting pattern)

// Copyright (c) 2016-2020 Aedan Cullen and/or Evolution Robotics.

import com.qualcomm.robotcore.hardware.DcMotor;

public class AutopilotTrackerQuadFullOdo extends AutopilotTracker {

    private DcMotor xF;
    private DcMotor xB;
    private DcMotor yL;
    private DcMotor yR;

    public boolean invertXF;
    public boolean invertXB;
    public boolean invertYL;
    public boolean invertYR;

    long xencF;
    long xencB;
    long yencL;
    long yencR;
    double ticksPerUnit;

    private double[] robotPosition = new double[3];
    private double[] robotAttitude = new double[3];

    private double deltaX;
    private double deltaY;
    private double deltaH;

    private double xRadius;
    private double yRadius;
    private double xTheta;
    private double yTheta;


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

    //
    // Set xRadius to the position in the y-direction of the x-encoder relative to the robot center
    // Set yRadius to the position in the x-direction of the y-encoders relative to the robot center (symmetric)
    // Set xOffset to the position in the x-direction of the x-encoder relative to the robot center
    // Set yOffset to the position in the y-direction of the y-encoders relative to the robot center (symmetric)
    //
    public AutopilotTrackerQuadFullOdo(DcMotor xF, DcMotor xB, DcMotor yL, DcMotor yR, double xRadius, double yRadius, double xOffset, double yOffset, double ticksPerUnit) {
        this.xF = xF;
        this.xB = xB;
        this.yL = yL;
        this.yR = yR;
        this.ticksPerUnit = ticksPerUnit;
        this.xRadius = xRadius;
        this.yRadius = yRadius;
        this.xTheta = Math.atan(xOffset / xRadius);
        this.yTheta = Math.atan(yOffset / yRadius);
    }

    public AutopilotTrackerQuadFullOdo(DcMotor xF, DcMotor xB, DcMotor yL, DcMotor yR, double xRadius, double yRadius, double ticksPerUnit) {
        this.xF = xF;
        this.xB = xB;
        this.yL = yL;
        this.yR = yR;
        this.ticksPerUnit = ticksPerUnit;
        this.xRadius = xRadius;
        this.yRadius = yRadius;
    }

    public void setInverts(boolean invertXF, boolean invertXB, boolean invertYL, boolean invertYR) {
        this.invertXF = invertXF;
        this.invertXB = invertXB;
        this.invertYL = invertYL;
        this.invertYR = invertYR;
    }


    public void update() {

        long ticksXF = xF.getCurrentPosition();
        long ticksXB = xB.getCurrentPosition();
        long ticksYL = yL.getCurrentPosition();
        long ticksYR = yR.getCurrentPosition();

        double xFval = ((double)(ticksXF - xencF) / ticksPerUnit);
        double xBval = ((double)(ticksXB - xencB) / ticksPerUnit);
        double yLval = ((double)(ticksYL - yencL) / ticksPerUnit);
        double yRval = ((double)(ticksYR - yencR) / ticksPerUnit);

        xencF = ticksXF;
        xencB = ticksXB;
        yencL = ticksYL;
        yencR = ticksYR;

        if (invertXF) {xFval = -xFval;}
        if (invertXB) {xBval = -xBval;}
        if (invertYL) {yLval = -yLval;}
        if (invertYR) {yRval = -yRval;}

        double unitsTurnY = (yRval - yLval) / 2.0; // CCW rotation is positive
        double dAY = (unitsTurnY / Math.cos(yTheta)) / yRadius;

        double unitsTurnX = (xBval - xFval) / 2.0; // CCW rotation is positive
        double dAX = (unitsTurnX / Math.cos(xTheta)) / xRadius;

        double dA = (dAX + dAY) / 2.0;

        double unitsTranslateY = (yLval + yRval) / 2.0;
        double unitsTranslateX = (xFval + xBval) / 2.0;

        deltaX = unitsTranslateX; deltaY = unitsTranslateY; deltaH = dA;

        double[] translationDelta = new double[] {unitsTranslateX, unitsTranslateY, 0};

        robotAttitude[0] += dA;
        if (robotAttitude[0] < -Math.PI) {robotAttitude[0] += 2*Math.PI;}
        if (robotAttitude[0] > Math.PI) {robotAttitude[0] -= 2*Math.PI;}

        robotPosition = transform(robotPosition, translationDelta, robotAttitude);
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

    public double getDeltaX() {
        return deltaX;
    }
    public double getDeltaY() {
        return deltaY;
    }
    public double getDeltaH() {
        return deltaH;
    }
    public double getDeltaPos() {
        return Math.sqrt(Math.pow(getDeltaX(), 2) + Math.pow(getDeltaY(), 2));
    }

}
