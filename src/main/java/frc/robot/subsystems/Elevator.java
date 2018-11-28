package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.RobotMap;
import frc.robot.util.PositionController;
import frc.robot.util.SubsystemModule;

/**
 *
 */
/*

public class Elevator extends SubsystemModule {

	public Spark elevatorMotor = new Spark(RobotMap.p_elevatorMotor);

	public DigitalInput limitSwitchElevatorBottom = new DigitalInput(6);// false if carriage at bottom
	public DigitalInput limitSwitchElevatorTop = new DigitalInput(7); // false if at top

	public Encoder elevatorEncoder = new Encoder(RobotMap.p_elevatorEncoderA, RobotMap.p_elevatorEncoderB, true,
			EncodingType.k4X);// measures carriage height

	public boolean controllerEnabled = false;
	
	public Elevator() {
		
		elevatorEncoder.reset(); 
		elevatorEncoder.setReverseDirection(false);

	}
	
	public void run() {
		if (controllerEnabled)
			ElevatorController.runController();
	}
	
	public PositionController ElevatorController = new PositionController(0.04, 0.001, 0) {
		@Override
		public void updateValues() {
			this.currentPosition = elevatorEncoder.getDistance();
		}
		
		@Override
		public void setOutput(double output) {
			elevatorMotor.set(output);
		}
		
	};

	public void registerCommands() {

	}

	public void initDefaultCommand() {

	}

	public void goToPosition(double height) {

	}

}

*/