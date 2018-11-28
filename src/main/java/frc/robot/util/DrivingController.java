package frc.robot.util;

import java.util.ArrayList;

public abstract class DrivingController {

	public double currentX = 0;
	public double currentY = 0;
	public double targetX = 0;
	public double targetY = 0;
	public double deltaX = 0;
	public double deltaY = 0;
	double targetAngle = 0;
	double deltaToAngle = 0;
	double deltaToDistance = 0;
	public double currentAngle = 0;

	//Angle and velocity controllers
	private PID anglePID = new PID(0.035, 0.0, 0.003);
	private PID velocityPID = new PID(0.6, 0.001, 0.0);

	private SplineFactory currentSpline;

	private int ahead = 1;

	// Run function for Driving Controller uses distance and angle controllers
	public void run() {
		updateVariables();

		anglePID.setOutputLimits(-1, 1);
		anglePID.setMaxIOutput(0.7);

		this.deltaX = targetX - currentX;
		this.deltaY = targetY - currentY;

		angleDisplacement();
		distanceDisplacement();

		driveRobot(velocityPID.getOutput(-this.ahead * this.deltaToDistance, 0),
				anglePID.getOutput(-this.deltaToAngle, 0));

	}

	// Abstract functions to move and get position of the robot
	public abstract void updateVariables();

	public abstract void driveRobot(double power, double pivot);

	// Set the desired position for the robot
	public void setSpline(double x1, double x2, double x3, double x4, double y1, double y2, double y3, double y4,
	double acceleration, double velocity) {
		this.currentSpline = new SplineFactory(x1, x2, x3, x4, y1, y2, y3, y4, acceleration, velocity);
	}

	// Check the angle difference to point robot at the target
	public void angleDisplacement() {
		if (this.deltaY == 0) {
			if (this.deltaX > 0) {
				this.targetAngle = 0;
			} else {
				this.targetAngle = 180;
			}
		} else if (this.deltaX == 0) {
			if (this.deltaY > 0) {
				this.targetAngle = 90;
			} else {
				this.targetAngle = 270;
			}
		}

		if (this.deltaX < 0) {
			this.targetAngle = (Math.atan(this.deltaY / this.deltaX) / Math.PI * 180) + 180;
		} else if (this.deltaY > 0) {
			this.targetAngle = (Math.atan(this.deltaY / this.deltaX) / Math.PI * 180);
		} else {
			this.targetAngle = (Math.atan(this.deltaY / this.deltaX) / Math.PI * 180) + 360;
		}

		this.deltaToAngle = getDifferenceInAngleDegrees(this.currentAngle, this.targetAngle);

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

	// Check distance displacement from current position to target
	public void distanceDisplacement() {
		this.deltaToDistance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
	}
}
