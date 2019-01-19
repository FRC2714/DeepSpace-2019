package frc.robot.util;

public abstract class PositionController {
	double posP, posI, posD, velP, velI, velD, velF;

	PID positionPID = new PID(posP, posI, posD);

	protected double currentPosition, setpoint;

	public PositionController(double posP, double posI, double posD) {

		this.posP = posP;
		this.posI = posI;
		this.posD = posD;

	}

	// This will be redefined to update the current value in the controller
	public abstract void updateValues();

	// This will be redefined to update the output in the subsystem
	public abstract void setOutput(double output);

	public void reset() {
		positionPID.reset();
	}

	public void setSetpoint(double setpoint) {
		this.setpoint = setpoint;
	}

	// This will be called inside the run function of the parent subsystem
	public void runController() {
		updateValues();
		double output = 0;

		// Perform necessary control function here
		output = positionPID.getOutput(this.currentPosition, this.setpoint);

		setOutput(output);
	}

}
