package frc.robot.util;

public abstract class Odometer {
	
	public double headingAngle;
	double lastHeadingAngle = 0;
	double direction;

	public double startOffset = 0;
	
	protected double leftDistance, leftPos;
	double lastLeftPos = 0;
	
	protected double rightDistance, rightPos;
	double lastRightPos = 0;
	
	double hypotenuseDistance;
	
	double change_x, change_y;
	public double current_x, current_y;
	
	public double currentVelocity;

	public Odometer(double startX, double startY, double startOffset) {
		
		this.current_x = startX;
		this.current_y = startY;
		this.startOffset = startOffset;
	}
	
	public void reset() {
		this.current_x = 0;
		this.current_y = 0;
	}
	
	//Update with getFusedHeading, leftPos, rightPos, and velocity (0.5 * rates)
	public abstract void updateEncodersAndHeading();
	
	// integrate the position of the robot using the distance from each encoder.
	public void integratePosition() {
		
		updateEncodersAndHeading();

		this.headingAngle = this.headingAngle + this.startOffset;
		if (this.headingAngle > 360)
			this.headingAngle -= 360;

		if (this.headingAngle < 0)
			this.headingAngle += 360;


		leftDistance = (leftPos - lastLeftPos);
		rightDistance = (rightPos - lastRightPos);

		hypotenuseDistance = (leftDistance + rightDistance) / 2;

//		change_x = (Math.cos((headingAngle * Math.PI) / 180) * hypotenuseDistance);
//		change_y = (Math.sin((headingAngle * Math.PI) / 180) * hypotenuseDistance);

		change_x = (Math.cos(Math.toRadians(headingAngle)) * hypotenuseDistance);
		change_y = (Math.sin(Math.toRadians(headingAngle)) * hypotenuseDistance);
		
		current_x = current_x + change_x;
		current_y = current_y + change_y;

		lastHeadingAngle = headingAngle;
		lastLeftPos = leftPos;
		lastRightPos = rightPos;

	}
}
