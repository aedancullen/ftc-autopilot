

package com.evolutionftc.autopilot;



import org.firstinspires.ftc.robotcore.external.Telemetry;



// Copyright (c) 2016-2018 Aedan Cullen and/or Evolution Robotics.


public class AutopilotHost {

    Telemetry telemetry;

    public enum NavigationStatus {RUNNING, ORIENTING, STOPPED};

    private NavigationStatus navigationStatus = NavigationStatus.STOPPED;

    private double basePower;
    private double lowestPower;
    private double powerGain;
    private double powerGainX;
    private double powerGainY;

    private double navigationHalfway;
    private double navigationHalfwayX;
    private double navigationHalfwayY;
    private boolean rampUp;
    private boolean rampDown;
    private boolean useOrientation;

    private double[] navigationTarget = new double[3];
    private double orientationTarget;
    private double steeringGain;
    private double[] accuracyThreshold = new double[3];
    private double orientationThreshold;

    private double[] robotAttitude = new double[3];

    private double[] robotPosition = new double[3];

    private boolean[] navigationTargetInverts = null;
    private boolean orientationTargetInvert = false;

    public AutopilotHost(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public boolean getOrientationTargetInvert() {
        return this.orientationTargetInvert;
    }

    public void setOrientationTargetInvert(boolean orientationTargetInvert) {
        this.orientationTargetInvert = orientationTargetInvert;
    }

    public boolean[] getNavigationTargetInverts() {
        return this.navigationTargetInverts;
    }

    public void setNavigationTargetInverts(boolean[] navigationTargetInverts) {
        this.navigationTargetInverts = navigationTargetInverts;
    }

    public void telemetryUpdate() {
        telemetry.addData("* AutopilotHost", "\n" +
        "\t status:  " + navigationStatus.toString().toLowerCase() + "\n" +
        "\t target:  " + round(navigationTarget[0]) + ",  " + round(navigationTarget[1]) + ",  " + round(navigationTarget[2]) + "\n" +
        "\t position:  " + round(robotPosition[0]) + ",  " + round(robotPosition[1]) + ",  " + round(robotPosition[2]) + "\n" +
        "\t attitude:  " + round(robotAttitude[0]) + ",  " + round(robotAttitude[1]) + ",  " + round(robotAttitude[2]));
        //telemetry.update();
    }

    public void communicate(AutopilotTracker tracker) {
        tracker.setRobotAttitude(robotAttitude);
        tracker.setRobotPosition(robotPosition);
        tracker.update();

        robotAttitude = tracker.getRobotAttitude();
        robotPosition = tracker.getRobotPosition();
    }

    public NavigationStatus getNavigationStatus() {
        return navigationStatus;
    }

    public void setNavigationStatus(NavigationStatus navigationStatus) {
        this.navigationStatus = navigationStatus;

    }

    public void setNavigationTarget(AutopilotSegment target) {
        setNavigationTarget(target.navigationTarget, target.orientationTarget, target.steeringGain, target.accuracyThreshold, target.orientationThreshold, target.basePower, target.lowestPower, target.rampUp, target.rampDown, target.useOrientation);
    }

    private void applyOrientationTargetInvert() {
        this.orientationTarget = (Math.PI*2) - this.orientationTarget;
    }

    private void applyNavigationTargetInverts() {
        for (int i=0; i<3; i++){
            if (this.navigationTargetInverts[i]) {
                this.navigationTarget[i] = -this.navigationTarget[i];
            }
        }
    }

    public void setNavigationTarget(double[] navigationTarget, double orientationTarget, double steeringGain, double[] accuracyThreshold, double orientationThreshold, double basePower, double lowestPower, boolean rampUp, boolean rampDown, boolean useOrientation) {
        this.navigationTarget = navigationTarget;
        this.orientationTarget = orientationTarget;
        this.steeringGain = steeringGain;
        this.accuracyThreshold = accuracyThreshold;
        this.orientationThreshold = orientationThreshold;
        this.basePower = basePower;
        this.lowestPower = lowestPower;
        this.rampUp = rampUp;
        this.rampDown = rampDown;
        this.useOrientation = useOrientation;
        double distX = navigationTarget[0] - robotPosition[0];
        double distY = navigationTarget[1] - robotPosition[1];
        double dist = Math.sqrt(Math.pow(distX, 2) + Math.pow(distY, 2));
        navigationHalfway = dist / 2;
        navigationHalfwayX = Math.abs(distX) / 2.0;
        navigationHalfwayY = Math.abs(distY) / 2.0;
        this.powerGain = (basePower - lowestPower) / navigationHalfway;
        this.powerGainX = (basePower - lowestPower) / navigationHalfwayX;
        this.powerGainY = (basePower - lowestPower) / navigationHalfwayY;

        if (this.navigationTargetInverts != null) {
            this.applyNavigationTargetInverts();
        }

        if (this.orientationTargetInvert) {
            this.applyOrientationTargetInvert();
        }

    }

    public double[] getNavigationTarget() {
        return navigationTarget;
    }

    public double[] getRobotAttitude() {
        return robotAttitude;
    }

    public double[] getRobotPosition() {
        return robotPosition;
    }

    public void setRobotPosition(double[] position) {
        robotPosition = position;
    }

    private static boolean hasReached(double param1, double param2, double threshold) {
        return (Math.abs(param2 - param1) < threshold);
    }

    private static double round(double in) {
        double roundOff = (double) Math.round(in * 100) / 100;
        return roundOff;
    }

    double nTimesStable = 0;

    public double[] navigationTickDifferential() {

        if ( // State transition case from RUNNING
        hasReached(robotPosition[0], navigationTarget[0], accuracyThreshold[0]) &&
        hasReached(robotPosition[1], navigationTarget[1], accuracyThreshold[1]) &&
        hasReached(robotPosition[2], navigationTarget[2], accuracyThreshold[2]) &&
        navigationStatus == NavigationStatus.RUNNING
        )
        {
            if (useOrientation) {
                navigationStatus = NavigationStatus.ORIENTING;
                nTimesStable = 0;
            }
            else {
                navigationStatus = NavigationStatus.STOPPED;
            }
        }


        else if (navigationStatus == NavigationStatus.ORIENTING) {
            if (hasReached(Math.abs(robotAttitude[0]), Math.abs(orientationTarget), orientationThreshold)) {nTimesStable++;}
            if (nTimesStable > 3) {
                navigationStatus = NavigationStatus.STOPPED;
            }
        }

        // the above transitions must be handled NOW
        if (navigationStatus == NavigationStatus.RUNNING) { // State action case for RUNNING
            double distX = navigationTarget[0] - robotPosition[0];
            double distY = navigationTarget[1] - robotPosition[1];
            double dist = Math.sqrt(Math.pow(distX, 2) + Math.pow(distY, 2));
            double powerAdj = (dist - navigationHalfway) * powerGain;
            if (powerAdj > 0 && !rampUp) {
                powerAdj = 0;
            }
            if (powerAdj < 0 && !rampDown) {
                powerAdj = 0;
            } else if (powerAdj < 0 && rampDown) {
                powerAdj *= -1;
            }

            double attitude = robotAttitude[0];
            double targAngle = -Math.atan(distX / distY);
            if (distY < 0) { // accommodate atan's restricted-range output, and expand it accordingly
                targAngle += Math.PI;
            }

            double angle = targAngle - attitude;


            if (angle > Math.PI) {
                angle = -(Math.PI * 2 - angle);
            }
            if (angle < -Math.PI) {
                angle = -(-Math.PI * 2 - angle);
            }

            if (Math.abs(angle) < Math.PI / 2) { // Drive forward
                double chosenPower = Math.max((basePower - powerAdj), lowestPower);
                double powerLeft = chosenPower - (angle * steeringGain);
                double powerRight = chosenPower + (angle * steeringGain);
                powerLeft = Math.min(powerLeft, chosenPower);
                powerRight = Math.min(powerRight, chosenPower);
                powerLeft = Math.max(powerLeft, -chosenPower);
                powerRight = Math.max(powerRight, -chosenPower);
                return new double[]{powerLeft, powerRight};
            } else {
                // Calculate the angle with respect to the back of the robot.

                if (angle > 0) {
                    angle = Math.PI - angle;
                } else {
                    angle = -Math.PI - angle;
                }

                // Drive backward
                // Note that we swap min and max, use -basePower, -lowestPower, and reverse steering operations
                double chosenPower = Math.min((-basePower + powerAdj), -lowestPower);
                double powerLeft = chosenPower + (angle * steeringGain);
                double powerRight = chosenPower - (angle * steeringGain);
                // also note that we must compare to -1
                powerLeft = Math.max(powerLeft, chosenPower);
                powerRight = Math.max(powerRight, chosenPower);
                powerLeft = Math.min(powerLeft, -chosenPower);
                powerRight = Math.min(powerRight, -chosenPower);
                return new double[]{powerLeft, powerRight};
            }
        }

        else if (navigationStatus == NavigationStatus.ORIENTING) { // State action case for ORIENTING
            double attitude = robotAttitude[0];

            double targAngle = orientationTarget;

            double angle = targAngle - attitude;


            if (angle > Math.PI) {
                angle = -(Math.PI * 2 - angle);
            }
            if (angle < -Math.PI) {
                angle = -(-Math.PI * 2 - angle);
            }

            double powerLeft =  -(angle * steeringGain);
            double powerRight = (angle * steeringGain);

            if (powerLeft > 0) {
                powerLeft = Math.min(powerLeft, basePower);
                powerLeft = Math.max(powerLeft, lowestPower);
            }
            else {
                powerLeft = Math.max(powerLeft, -basePower);
                powerLeft = Math.min(powerLeft, -lowestPower);
            }

            if (powerRight > 0) {
                powerRight = Math.min(powerRight, basePower);
                powerRight = Math.max(powerRight, lowestPower);
            }
            else {
                powerRight = Math.max(powerRight, -basePower);
                powerRight = Math.min(powerRight, -lowestPower);
            }

            return new double[]{powerLeft, powerRight};

        }

        else { // Navigation status must be STOPPED
            return new double[2];
        }

    }

    public double[] navigationTickFoursides() {

        if ( // State transition case from RUNNING
        hasReached(robotPosition[0], navigationTarget[0], accuracyThreshold[0]) &&
        hasReached(robotPosition[1], navigationTarget[1], accuracyThreshold[1]) &&
        hasReached(robotPosition[2], navigationTarget[2], accuracyThreshold[2]) &&
        navigationStatus == NavigationStatus.RUNNING
        )
        {
            if (useOrientation) {
                navigationStatus = NavigationStatus.ORIENTING;
                nTimesStable = 0;
            }
            else {
                navigationStatus = NavigationStatus.STOPPED;
            }
        }


        else if (navigationStatus == NavigationStatus.ORIENTING) {
            if (hasReached(Math.abs(robotAttitude[0]), Math.abs(orientationTarget), orientationThreshold)) {nTimesStable++;}
            if (nTimesStable > 3) {
                navigationStatus = NavigationStatus.STOPPED;
            }
        }

        // the above transitions must be handled NOW
        if (navigationStatus == NavigationStatus.RUNNING) { // State action case for RUNNING
            double distX = navigationTarget[0] - robotPosition[0];
            double distY = navigationTarget[1] - robotPosition[1];
            double dist = Math.sqrt(Math.pow(distX, 2) + Math.pow(distY, 2));
            double powerAdj = (dist - navigationHalfway) * powerGain;
            if (powerAdj > 0 && !rampUp) {
                powerAdj = 0;
            }
            if (powerAdj < 0 && !rampDown) {
                powerAdj = 0;
            } else if (powerAdj < 0 && rampDown) {
                powerAdj *= -1;
            }

            double attitude = robotAttitude[0];
            double targAngle = -Math.atan(distX / distY);
            if (distY < 0) { // accommodate atan's restricted-range output, and expand it accordingly
                targAngle += Math.PI;
            }

            double angle = targAngle - attitude;


            if (angle > Math.PI) {
                angle = -(Math.PI * 2 - angle);
            }
            if (angle < -Math.PI) {
                angle = -(-Math.PI * 2 - angle);
            }

            boolean foursides_perpendicular_flip = false;
            if (Math.abs(angle) > Math.PI / 4 && Math.abs(angle) < 3 * Math.PI / 4) {
                // angle within right/left range
                foursides_perpendicular_flip = true;
                angle += Math.PI / 4; // Imagine the front of the robot is facing right
            }

            if (angle > Math.PI) {
                angle = -(Math.PI * 2 - angle);
            }
            if (angle < -Math.PI) {
                angle = -(-Math.PI * 2 - angle);
            }

            if (Math.abs(angle) < Math.PI / 2) {
                double chosenPower = Math.max((basePower - powerAdj), lowestPower);
                double powerLeft = chosenPower - (angle * steeringGain);
                double powerRight = chosenPower + (angle * steeringGain);
                powerLeft = Math.min(powerLeft, chosenPower);
                powerRight = Math.min(powerRight, chosenPower);
                powerLeft = Math.max(powerLeft, -chosenPower);
                powerRight = Math.max(powerRight, -chosenPower);
                return new double[]{powerLeft, powerRight};
            } else {

                if (angle > 0) {
                    angle = Math.PI - angle;
                } else {
                    angle = -Math.PI - angle;
                }

                double chosenPower = Math.min((-basePower + powerAdj), -lowestPower);
                double powerLeft = chosenPower + (angle * steeringGain);
                double powerRight = chosenPower - (angle * steeringGain);

                powerLeft = Math.max(powerLeft, chosenPower);
                powerRight = Math.max(powerRight, chosenPower);
                powerLeft = Math.min(powerLeft, -chosenPower);
                powerRight = Math.min(powerRight, -chosenPower);

                double b_foursides_perpendicular_flip = 0;
                if (foursides_perpendicular_flip) {b_foursides_perpendicular_flip = 1;}
                return new double[]{b_foursides_perpendicular_flip, powerLeft, powerRight};
            }
        }

        else if (navigationStatus == NavigationStatus.ORIENTING) { // State action case for ORIENTING
            double attitude = robotAttitude[0];

            double targAngle = orientationTarget;

            double angle = targAngle - attitude;


            if (angle > Math.PI) {
                angle = -(Math.PI * 2 - angle);
            }
            if (angle < -Math.PI) {
                angle = -(-Math.PI * 2 - angle);
            }

            double powerLeft =  -(angle * steeringGain);
            double powerRight = (angle * steeringGain);

            if (powerLeft > 0) {
                powerLeft = Math.min(powerLeft, basePower);
                powerLeft = Math.max(powerLeft, lowestPower);
            }
            else {
                powerLeft = Math.max(powerLeft, -basePower);
                powerLeft = Math.min(powerLeft, -lowestPower);
            }

            if (powerRight > 0) {
                powerRight = Math.min(powerRight, basePower);
                powerRight = Math.max(powerRight, lowestPower);
            }
            else {
                powerRight = Math.max(powerRight, -basePower);
                powerRight = Math.min(powerRight, -lowestPower);
            }

            return new double[]{0, powerLeft, powerRight};

        }

        else { // Navigation status must be STOPPED
            return new double[3];
        }

    }


    public double[] navigationTick() {

        if (
                hasReached(robotPosition[0], navigationTarget[0], accuracyThreshold[0]) &&
                        hasReached(robotPosition[1], navigationTarget[1], accuracyThreshold[1]) &&
                        hasReached(robotPosition[2], navigationTarget[2], accuracyThreshold[2]) &&
                        navigationStatus == NavigationStatus.RUNNING
                )
        {
            if (useOrientation) {
                navigationStatus = NavigationStatus.ORIENTING;
                nTimesStable = 0;
            }
            else {
                navigationStatus = NavigationStatus.STOPPED;
            }
        }


        else if (navigationStatus == NavigationStatus.ORIENTING) {
            if (hasReached(Math.abs(robotAttitude[0]), Math.abs(orientationTarget), orientationThreshold)) {nTimesStable++;}
            if (nTimesStable > 3) {
                navigationStatus = NavigationStatus.STOPPED;
            }
        }


        if (navigationStatus != NavigationStatus.STOPPED) {
            double distX = navigationTarget[0] - robotPosition[0];
            double distY = navigationTarget[1] - robotPosition[1];
            double dist = Math.sqrt(Math.pow(distX, 2) + Math.pow(distY, 2));
            double powerAdj = (dist - navigationHalfway) * powerGain;
            if (powerAdj > 0 && !rampUp) {
                powerAdj = 0;
            }
            if (powerAdj < 0 && !rampDown) {
                powerAdj = 0;
            } else if (powerAdj < 0 && rampDown) {
                powerAdj *= -1;
            }

            double attitude = robotAttitude[0];
            double translateTargAngle = -Math.atan(distX / distY);
            if (distY < 0) {
                translateTargAngle += Math.PI;
            }

            double translateAngle = translateTargAngle - attitude;


            if (translateAngle > Math.PI) {
                translateAngle = -(Math.PI * 2 - translateAngle);
            }
            if (translateAngle < -Math.PI) {
                translateAngle = -(-Math.PI * 2 - translateAngle);
            }

            double rotateTargAngle = orientationTarget;

            double rotateAngle = rotateTargAngle - attitude;

            if (rotateAngle > Math.PI) {
                rotateAngle = -(Math.PI * 2 - rotateAngle);
            }
            if (rotateAngle < -Math.PI) {
                rotateAngle = -(-Math.PI * 2 - rotateAngle);
            }


            double powerRot = 0;
            double powerY = 0;
            double powerX = 0;

            if (navigationStatus == navigationStatus.ORIENTING) {
                powerRot = -(rotateAngle * steeringGain); // positive is clockwise
                if (powerRot > 0) {
                    powerRot = Math.min(powerRot, basePower);
                    powerRot = Math.max(powerRot, lowestPower);
                }
                else {
                    powerRot = Math.max(powerRot, -basePower);
                    powerRot = Math.min(powerRot, -lowestPower);
                }
            }

            if (navigationStatus == navigationStatus.RUNNING) {
                double chosenPower = Math.max((basePower - powerAdj), lowestPower);
                powerY = Math.cos(translateAngle) * chosenPower;
                powerX = -Math.sin(translateAngle) * chosenPower;
            }

            return new double[]{powerX, powerY, powerRot};

        }

        else {
            return new double[3];
        }

    }

}
