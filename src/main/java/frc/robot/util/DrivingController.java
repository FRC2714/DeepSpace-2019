package frc.robot.util;

import java.util.ArrayList;

public abstract class DrivingController {

	// Angle and velocity controllers
	private PID samsonControl = new PID(0.01, 0.0, 0);
	private PID tangentialControl = new PID(0.0, 0.0, 0.0);

	// Coefficients for the kinematic control algorithm
	private double k2 = 0.0;
	private double k3 = 1.0;

	private ArrayList<MotionPose> controlPath = new ArrayList<MotionPose>();

	protected double currentX;
	protected double currentY;
	protected double currentAngle;
	protected double currentAverageVelocity;

	private double samsonOutput;
	private double tangentialOutput;

	private int iterator = 0;
	private double period;

	public DrivingController(double period) {

		// Set up period
		this.period = period;

	}

	// Run function for Driving Controller uses distance and angle controllers
	public void run() {

		// Update using abstracted functions from the calling class
		updateVariables();

		// Move to the next point in the spline
		next();

		// Use tangential correction and velocity control cascaded to control velocity and position.
		double orthogonalError = controlPath.get(iterator).getOrthogonalDisplacement(currentX, currentY);
		double tangentialError = controlPath.get(iterator).getTangentialDisplacement(currentX, currentY);
		
		double refVelocity = controlPath.get(iterator).velocity;
		double angleDifference = getDifferenceInAngleDegrees(currentAngle, controlPath.get(iterator).angle);

		double samsonCorrection2 = (k2 * orthogonalError * refVelocity) / angleDifference;
		double samsonCorrection3 = k3 * angleDifference;

		samsonOutput = samsonControl.getOutput(samsonCorrection2 + samsonCorrection3, 0);
		tangentialOutput = tangentialControl.getOutput(tangentialError, 0);

		driveRobot(refVelocity + tangentialOutput, -(samsonOutput / period));
	}

	// Abstract functions to move and get position of the robot
	public abstract void updateVariables();
	public abstract void driveRobot(double power, double pivot);

	// Angle difference utility
	private double getDifferenceInAngleDegrees(double from, double to) {
		return boundAngleNeg180to180Degrees(from - to);
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

	// Set the desired position for the robot
	public void addSpline(double x1, double x2, double x3, double x4, double y1, double y2, double y3, double y4,
			double acceleration, double maxVelocity, double startVelocity, double endVelocity, boolean forwards) {
		new SplineFactory(this.controlPath, this.period, x1, x2, x3, x4, y1, y2, y3, y4, acceleration, maxVelocity,
				startVelocity, endVelocity, forwards);
	}

	// Move to next motion pose in the sequence
	public void next() {
		this.iterator++;
	}

	// Calculate distance
	public double distanceCalc(double x1, double x2, double y1, double y2) {
		return Math.pow((Math.pow(x2 - x1, 2)) + (Math.pow(y2 - y1, 2)), 0.5);
	}
}
