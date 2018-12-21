package frc.robot.util;

import java.util.ArrayList;

public class SplineFactory {
    private double x1, x2, x3, x4, y1, y2, y3, y4, acceleration, velocity;
    // private ArrayList<Double> xValues = new ArrayList<Double>();
    // private ArrayList<Double> yValues = new ArrayList<Double>();

    private double tolerance = 0.0000001;
    private double tStep = 0.001;
    private double period = 0.0005;

    public SplineFactory(ArrayList<Double> xValues, ArrayList<Double> yValues, double period, double x1, double x2, double x3, double x4, double y1, double y2, double y3,
            double y4, double acceleration, double maxVelocity, double startVelocity, double endVelocity) {

        this.period = period;

        this.x1 = x1;
        this.x2 = x2;
        this.x3 = x3;
        this.x4 = x4;
        this.y1 = y1;
        this.y2 = y2;
        this.y3 = y3;
        this.y4 = y4;

        this.acceleration = acceleration * period;
        this.velocity = maxVelocity * period;

        // Fill point buffer
        double backT = 1;
        double frontT = 0;
        int placement = xValues.size();
        double currentFrontVelocity = startVelocity * this.period;
        double currentBackVelocity = endVelocity * this.period;

        while (frontT < backT) {

            // Discrete time form trapezoidal:
            // PT = PT + VT +1/2A
            // VT = VT+A
            // where:
            // P0 and V0, are the starting position and velocities
            // PT and VT, are the position and velocity at time T
            // A is the profile acceleration

            if (currentFrontVelocity < this.velocity) {
                currentFrontVelocity += this.acceleration * this.period;
            }

            if (currentBackVelocity < this.velocity) {
                currentBackVelocity += this.acceleration * this.period;
            }

            // Find front and back position
            frontT = binaryFind(frontT, currentFrontVelocity, placement, xValues, yValues);
            backT = binaryFind(backT, -currentBackVelocity, placement + 1, xValues, yValues);
            placement++;

        }

    }

    double binaryFind(double startT, double distance, int location, ArrayList<Double> xValues, ArrayList<Double> yValues) {
        double internalT = startT;
        double tStep_modified = this.tStep;
        double inverted = 1;
        double direction = 1;
        double lastDirection = 1;
        if (distance < 0) {
            inverted = -1;
            direction = -1;
            lastDirection = -1;
        }

        double startX = quarticCalc(internalT, this.x1, this.x2, this.x3, this.x4);
        double startY = quarticCalc(internalT, this.y1, this.y2, this.y3, this.y4);
        double newX, newY;

        double currentDistance = 0;
        double targetDistance = Math.abs(distance);
        double distanceDelta = targetDistance - currentDistance;

        do {

            if (distanceDelta * inverted < 0) {
                // Past target
                direction = -1;
                if (direction != lastDirection) {
                    tStep_modified *= 0.5;
                }
            } else {
                // Not past target
                direction = 1;
                if (direction != lastDirection) {
                    tStep_modified *= 0.5;
                }
            }

            internalT += tStep_modified * direction;

            newX = quarticCalc(internalT, this.x1, this.x2, this.x3, this.x4);
            newY = quarticCalc(internalT, this.y1, this.y2, this.y3, this.y4);

            currentDistance = distanceCalc(startX, newX, startY, newY);

            lastDirection = direction;
            distanceDelta = targetDistance - currentDistance;

        } while (Math.abs(distanceDelta) > this.tolerance);

        xValues.add(location, newX);
        yValues.add(location, newY);

        return internalT;
    }

    // Calculate quartic spline point with 4 controls
    public double quarticCalc(double t, double c1, double c2, double c3, double c4) {
        return (Math.pow(1 - t, 3) * c1) + 3 * (Math.pow(1 - t, 2) * t * c2) + 3 * (Math.pow(t, 2) * (1 - t) * c3)
                + (Math.pow(t, 3) * c4);
    }

    // Calculate distance
    public double distanceCalc(double x1, double x2, double y1, double y2) {
        return Math.pow((Math.pow(x2 - x1, 2)) + (Math.pow(y2 - y1, 2)), 0.5);
    }

}