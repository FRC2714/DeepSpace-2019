package frc.robot.util;

public abstract class Odometer {

	protected double headingAngle;
	private double startOffset = 0;

	private double leftDistance;
	protected double leftPos;
	private double lastLeftPos = 0;

	private double rightDistance;
	protected double rightPos;
	private double lastRightPos = 0;

	private double hypotenuseDistance;

	private double change_x, change_y;
	private double current_x, current_y;

	public Odometer(double startX, double startY, double startOffset) {
		this.current_x = startX;
		this.current_y = startY;
		this.startOffset = startOffset;
	}

	public void reset() {
		current_x = 0;
		current_y = 0;
	}

	// Update with getFusedHeading, leftPos, rightPos, and velocity (0.5  rates)
	public abstract void updateEncodersAndHeading();

	// Integrate the position of the robot using the distance from each encoder.
	public void integratePosition() {

		updateEncodersAndHeading();

		headingAngle = headingAngle + startOffset;

		if (headingAngle > 360)
			headingAngle -= 360;

		if (headingAngle < 0)
			headingAngle += 360;

		leftDistance = (leftPos - lastLeftPos);
		rightDistance = (rightPos - lastRightPos);

		hypotenuseDistance = (leftDistance + rightDistance) / 2;

		change_x = (Math.cos(Math.toRadians(headingAngle)) * hypotenuseDistance);
		change_y = (Math.sin(Math.toRadians(headingAngle)) * hypotenuseDistance);

		current_x = current_x + change_x;
		current_y = current_y + change_y;

		lastLeftPos = leftPos;
		lastRightPos = rightPos;

	}

	public void setOffset(double offset) {
		startOffset = offset;
	}

	public double getCurrentX() {
		return current_x;
	}

	public double getCurrentY() {
		return current_y;
	}

	public double getHeadingAngle() {
		return headingAngle;
	}
}
