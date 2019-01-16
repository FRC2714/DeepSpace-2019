package frc.robot.util;

import java.util.ArrayList;

public abstract class DrivingController {

	// Angle and velocity controllers
	private PID orthogonalControl = new PID(0.01, 0.0, 0);
	private PID tangentialControl = new PID(0.0, 0.0, 0.0);
	private PID velocityControl = new PID(0.55, 0.0005, 0.0);

	// Coefficient for angular correction
	private double kCC = 25;

	private ArrayList<MotionPose> controlPath = new ArrayList<MotionPose>();

	protected double currentX;
	protected double currentY;
	protected double currentAngle;
	protected double currentAverageVelocity;

	public String stringout;

	private int iterator = 0;

	private double period;

	private int ahead = 1;

	public DrivingController(double period) {

		// Set up period
		this.period = period;

		this.velocityControl.setMaxIOutput(0.1);
		this.velocityControl.setOutputLimits(-1, 1);

	}

	// Run function for Driving Controller uses distance and angle controllers
	// Iterate over the points in the vector
	public void run() {
		updateVariables();

		// Move to the next point in the spline
		next();
		//System.out.println("des: " + ((this.controlPath.get(iterator).angle/Math.PI) * 180) + " curr: " + this.currentAngle);

		// Use tangential correction and velocity control cascaded to control velocity and position.
		// Currently, use square of orthogonal error as a correction to modify angle to reduce error
		// Need to update to full Samson control algorithm
		double tangentialOutput = tangentialControl.getOutput(-this.controlPath.get(iterator).getTangentialDisplacement(currentX, currentY), 0);
		double velocityOutput = velocityControl.getOutput(this.currentAverageVelocity, this.controlPath.get(iterator).velocity - 0);
		double orthogonalOutput = orthogonalControl.getOutput(getDifferenceInAngleDegrees(this.currentAngle, this.controlPath.get(iterator).angle), 0);

		this.stringout = "output: " + Double.toString(orthogonalOutput) + " currentAngleDel: " + Double.toString(getDifferenceInAngleDegrees(this.currentAngle, this.controlPath.get(iterator).angle));
		//System.out.println(getDifferenceInAngleDegrees(this.currentAngle, this.controlPath.get(iterator).angle));

		/*- (this.kCC * Math.pow(this.controlPath.get(iterator).getOrthogonalDisplacement(currentX, currentY), 2))*/

		driveRobot(velocityOutput, -orthogonalOutput);

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
