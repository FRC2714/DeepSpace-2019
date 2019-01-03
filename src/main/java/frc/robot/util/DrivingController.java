package frc.robot.util;

import java.util.ArrayList;

public abstract class DrivingController {

	// Angle and velocity controllers
	private PID orthogonalControl = new PID(0.035, 0.0, 0.003);
	private PID tangentialControl = new PID(0.2, 0.0001, 0.0001);
	private PID velocityControl = new PID(0.6, 0.001, 0.0);

	// Coefficient for angular correction
	private double kCC = 25;

	private ArrayList<MotionPose> controlPath;

	protected double currentX;
	protected double currentY;
	protected double currentAngle;
	protected double currentAverageVelocity;

	private int iterator = 0;

	private double period;

	private int ahead = 1;

	public DrivingController(double period) {

		// Set up period
		this.period = period;

	}

	// Run function for Driving Controller uses distance and angle controllers
	// Iterate over the points in the vector
	public void run() {
		updateVariables();

		// Move to the next point in the spline
		next();

		// Use tangential correction and velocity control cascaded to control velocity and position.
		// Currently, use square of orthogonal error as a correction to modify angle to reduce error
		// Need to update to full Samson control algorithm
		double tangentialOutput = tangentialControl.getOutput(-this.controlPath.get(iterator).getTangentialDisplacement(currentX, currentY), 0);
		double velocityOutput = velocityControl.getOutput(this.currentAverageVelocity, this.controlPath.get(iterator).velocity + tangentialOutput);
		double orthogonalOutput = orthogonalControl.getOutput(this.currentAngle, this.controlPath.get(iterator).angle - (this.kCC * Math.pow(this.controlPath.get(iterator).getOrthogonalDisplacement(currentX, currentY), 2)));

		driveRobot(velocityOutput, orthogonalOutput);

	}

	// Abstract functions to move and get position of the robot
	public abstract void updateVariables();

	public abstract void driveRobot(double power, double pivot);

	// Set the desired position for the robot
	public void addSpline(double x1, double x2, double x3, double x4, double y1, double y2, double y3, double y4,
			double acceleration, double maxVelocity, double startVelocity, double endVelocity, boolean forwards) {
		new SplineFactory(this.controlPath, this.period, x1, x2, x3, x4, y1, y2, y3,
				y4, acceleration, maxVelocity, startVelocity, endVelocity, forwards);
	}

	/*

	
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
	*/

	public void next() {
		this.iterator++;
	}

	// Calculate distance
	public double distanceCalc(double x1, double x2, double y1, double y2) {
		return Math.pow((Math.pow(x2 - x1, 2)) + (Math.pow(y2 - y1, 2)), 0.5);
	}
}
