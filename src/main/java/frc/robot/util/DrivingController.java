package frc.robot.util;

import java.util.ArrayList;

public abstract class DrivingController {

	// Angle and velocity controllers
	private PID anglePID = new PID(0.035, 0.0, 0.003);
	private PID velocityPID = new PID(0.6, 0.001, 0.0);

	private ArrayList<Double> xValues = new ArrayList<Double>();
	private ArrayList<Double> yValues = new ArrayList<Double>();

	protected double currentX;
	protected double currentY;
	protected double currentAngle;
	protected double currentAverageVelocity;

	private double deltaToAngle;
	private int lookaheadSegments;
	private int iterator = 0;

	private double period;

	private int ahead = 1;

	public DrivingController(double period) {

		// Configure the PID controllers
		anglePID.setOutputLimits(-1, 1);
		anglePID.setMaxIOutput(0.7);

		// Set up period
		this.period = period;

	}

	// Run function for Driving Controller uses distance and angle controllers
	// Iterate over the points in the vector
	public void run() {

		double velocityScalar, targetAngle;
		updateVariables();

		// Move to the next point in the spline
		next();

		// Calculate the next velocity scalar and then calculate the target angle
		velocityScalar = getVelocityScalar();
		targetAngle = getAngleVector(this.lookaheadSegments);
		getAngleCorrection(targetAngle);

		// Use velocity target and angle delta to drive
		// driveRobot(velocityPID.getOutput(this.currentAverageVelocity, this.ahead * velocityScalar),
		// 		anglePID.getOutput(-this.deltaToAngle, 0));
		driveRobot(velocityPID.getOutput(this.currentAverageVelocity, this.ahead * velocityScalar), 0);

	}

	// Abstract functions to move and get position of the robot
	public abstract void updateVariables();

	public abstract void driveRobot(double power, double pivot);

	// Set the lookahead time, this calculates the lookahead distance
	public void setLookaheadSegments(int segments) {
		this.lookaheadSegments = segments;
	}

	// Set the desired position for the robot
	public void addSpline(double x1, double x2, double x3, double x4, double y1, double y2, double y3, double y4,
			double acceleration, double maxVelocity, double startVelocity, double endVelocity) {
		SplineFactory newSpline = new SplineFactory(this.xValues, this.yValues, this.period, x1, x2, x3, x4, y1, y2, y3,
				y4, acceleration, maxVelocity, startVelocity, endVelocity);
	}
	
	public void addSplineEnd() {
		double xDelta = this.xValues.get(this.xValues.size()-1)-this.xValues.get(this.xValues.size()-2);
		double yDelta = this.yValues.get(this.yValues.size()-1)-this.yValues.get(this.yValues.size()-2);

		for(int i = 0; i < lookaheadSegments; i++){
			this.xValues.add(this.xValues.get(this.xValues.size()-1) + (xDelta * i));
			this.yValues.add(this.yValues.get(this.yValues.size()-1) + (yDelta * i));
		}
		
	}

	// Angle difference utility
	private double getDifferenceInAngleDegrees(double from, double to) {
		return boundAngleNeg180to180Degrees(from - to);
	}

	// Calculate correction angle
	private void getAngleCorrection(double targetAngle) {
		this.deltaToAngle = getDifferenceInAngleDegrees(this.currentAngle, targetAngle);

		if (this.deltaToAngle < -90) {
			this.deltaToAngle += 180;
			this.ahead = -1;
		} else if (this.deltaToAngle > 90) {
			this.deltaToAngle -= 180;
			this.ahead = -1;
		} else {
			this.ahead = 1;
		}
	}

	// Keep it between 180 degrees
	private double boundAngleNeg180to180Degrees(double angle) {
		while (angle >= 180.0) {
			angle -= 360.0;
		}
		while (angle < -180.0) {
			angle += 360.0;
		}
		return angle;
	}

	public double getAngleVector(int segments) {
		double deltaX, deltaY;
		double angleVector;

		deltaX = this.xValues.get(this.iterator + segments) - this.xValues.get(this.iterator);
		deltaY = this.yValues.get(this.iterator + segments) - this.yValues.get(this.iterator);

		if (deltaY == 0) {
			if (deltaX > 0) {
				angleVector = 0;
			} else {
				angleVector = 180;
			}
		} else if (deltaX == 0) {
			if (deltaY > 0) {
				angleVector = 90;
			} else {
				angleVector = 270;
			}
		}

		if (deltaX < 0) {
			angleVector = (Math.atan(deltaY / deltaX) / Math.PI * 180) + 180;
		} else if (deltaY > 0) {
			angleVector = (Math.atan(deltaY / deltaX) / Math.PI * 180);
		} else {
			angleVector = (Math.atan(deltaY / deltaX) / Math.PI * 180) + 360;
		}

		return angleVector;
	}

	public double getVelocityScalar() {
		return distanceCalc(this.xValues.get(this.iterator), this.xValues.get(this.iterator + 1),
				this.yValues.get(this.iterator), this.yValues.get(this.iterator + 1)) / this.period;
	}

	public void next() {
		this.iterator++;
	}

	// Calculate distance
	public double distanceCalc(double x1, double x2, double y1, double y2) {
		return Math.pow((Math.pow(x2 - x1, 2)) + (Math.pow(y2 - y1, 2)), 0.5);
	}
}
