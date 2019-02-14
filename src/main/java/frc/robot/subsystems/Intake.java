package frc.robot.subsystems;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.util.SubsystemCommand;
import frc.robot.util.SubsystemModule;

public class Intake extends SubsystemModule {

	// Intake Motors
	private CANSparkMax cargoMotor = new CANSparkMax(9, MotorType.kBrushless);
	private CANSparkMax hatchplatePump = new CANSparkMax(10, MotorType.kBrushless);

	// Hatchplate Servo
	private Servo hatchplateServo = new Servo(0);
	private Servo pumpServo = new Servo(1);

	// Maximum currents for cargo and hatch intakes
	private double cargoCurrentThreshold = 20;
	private double hatchCurrentThreshold = -20;
	private double pumpCurrentThreshold = 5.5;

	// Intake States - Public so Arm can access the states for state-based logic
	private boolean cargoState = false;
	private boolean hatchState = false;

	// Hatch intake types
	private boolean hatchFloor = false;
	private boolean hatchStation = false;

	// Intake position
	private boolean atPosition = false;

    public Intake() { 
		registerCommands(); // Puts commands onto the hashmap
	}

	/**
	 * Raises the hatchplate into tucked position
	 */
	public void hatchplateUp() {
		hatchplateServo.set(1);
	}

	/**
	 * Lowers the hatchplate into active position
	 */
	public void hatchplateDown() {
		hatchplateServo.set(0.47);
	}

	/**
	 * Puts pump servo into hatch intake mode
	 */
	public void pumpHatch() {
		pumpServo.set(0.05);
	}

	/**
	 * Puts pump servo into release mode
	 */
	public void pumpRelease() {
		pumpServo.set(0.5);
	}

	/**
	 * Puts pump servo into climb mode
	 */
	public void pumpClimb() {
		pumpServo.set(0.95);
	}

	public boolean getCargoState() {
		return cargoState;
	}

	public boolean getHatchState() {
		return hatchState;
	}

	public boolean getHatchFloor() {
		return hatchFloor;
	}

	public boolean getHatchStation() {
		return hatchStation;
	}

	public void setAtPosition(boolean atPosition) {
		this.atPosition = atPosition;
		System.out.println("setAt: " + atPosition);
	}

	@Override
	public void run() {
    }

	@Override
	public void registerCommands() {

		new SubsystemCommand(this.registeredCommands, "servo_down") {

			@Override
			public void initialize() {
				hatchplateDown();
			}

			@Override
			public void execute() {

			}

			@Override
			public boolean isFinished() {
				return false;
			}

			@Override
			public void end() {

			}
		};

		new SubsystemCommand(this.registeredCommands, "servo_up") {

			@Override
			public void initialize() {
				hatchplateUp();
			}

			@Override
			public void execute() {

			}

			@Override
			public boolean isFinished() {
				return false;
			}

			@Override
			public void end() {

			}
		};

        new SubsystemCommand(this.registeredCommands, "cargo_intake") {
			ArrayList<Double> currents = new ArrayList<Double>();
			double avgCurrent = 0;
			double maxSize = 25;

			boolean intaking;

			@Override
			public void initialize() {
				intaking = false;
			}

			@Override
			public void execute() {
				currents.add(0, cargoMotor.getOutputCurrent());

				if (!intaking && atPosition) {
					cargoMotor.set(0.75);
					intaking = true;
				}
				else if(currents.size() <= maxSize) {
					avgCurrent += currents.get(0) / maxSize;
				}
				else {
					if (Math.abs(avgCurrent) > cargoCurrentThreshold) { cargoState = true; }

					avgCurrent += (currents.get(0) - currents.get(currents.size() - 1)) / maxSize;
					currents.remove(currents.size() - 1);
				}
			}

			@Override
			public boolean isFinished() {
				return cargoState || hatchState;
			}

			@Override
			public void end() {
				cargoMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
				currents = new ArrayList<Double>();

				if (cargoState && !hatchState) {
					cargoMotor.set(0.05);
				} else {
					cargoMotor.set(0);
				}
			}
		};

        new SubsystemCommand(this.registeredCommands, "hatch_floor_intake") {
			ArrayList<Double> currents = new ArrayList<Double>(0);
			double avgCurrent = 0;
			double maxSize = 25;

			boolean intaking;

			@Override
			public void initialize() {
				intaking = false;
			}

			@Override
			public void execute() {
				currents.add(0, cargoMotor.getOutputCurrent());

				if (!intaking && atPosition) {
					cargoMotor.set(-0.75);
					hatchplatePump.set(1);
					pumpHatch();
					intaking = true;
				}
				else if(currents.size() <= maxSize) {
					avgCurrent += currents.get(0) / maxSize;
				}
				else {
					if (Math.abs(avgCurrent) > hatchCurrentThreshold) {
						hatchState = true;
					}

					avgCurrent += (currents.get(0) - currents.get(currents.size() - 1)) / maxSize;
					currents.remove(currents.size() - 1);
				}
			}

			@Override
			public boolean isFinished() {
				return cargoState || hatchState;
			}

			@Override
			public void end() {
				cargoMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
				cargoMotor.set(0);

				if (!cargoState && hatchState) {
					hatchplateDown();

					hatchFloor = true;
					hatchStation = false;
				}
				else {
					hatchplatePump.set(0);
				}
			}
		};

        new SubsystemCommand(this.registeredCommands, "hatch_station_intake") {
			ArrayList<Double> currents = new ArrayList<Double>();
			double avgCurrent = 0;
			double maxSize = 25;

			boolean intaking;

			@Override
			public void initialize() {
				intaking = false;
			}

			@Override
			public void execute() {
				currents.add(0, hatchplatePump.getOutputCurrent());
				
				if(!intaking && atPosition) {
					hatchplatePump.set(1);
					hatchplateDown();
					pumpHatch();
					intaking = true;
				}
				if(currents.size() <= maxSize) {
					avgCurrent += currents.get(0) / maxSize;
				}
				else {
					avgCurrent += (currents.get(0) - currents.get(currents.size() - 1)) / maxSize;
					currents.remove(currents.size() - 1);

					if(Math.abs(avgCurrent) < pumpCurrentThreshold) { hatchState = true; }
				}
			}

			@Override
			public boolean isFinished() {
				return cargoState || hatchState;
			}

			@Override
			public void end() {
				cargoMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

				if (!cargoState && hatchState) {
					hatchFloor = false;
					hatchStation = true;
				}
				else { hatchplatePump.set(0); }

				currents = new ArrayList<Double>();
			}
		};

        new SubsystemCommand(this.registeredCommands, "extake") {
			int counter;

			@Override
			public void initialize() {
				counter = 0;

				if (cargoState && !hatchState) {
					cargoMotor.set(-0.5);
				} else if (!cargoState && hatchState) {
					pumpRelease();
					hatchplatePump.set(0);
				}
			}

			@Override
			public void execute() {

			}

			@Override
			public boolean isFinished() {
				counter++;
				return counter < 25;
			}

			@Override
			public void end() {
				cargoMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

				hatchplateUp();
				cargoMotor.set(0);

				cargoState = false;
				hatchState = false;

				hatchFloor = false;
				hatchStation = false;
			}
		};

		new SubsystemCommand(this.registeredCommands, "set_cargo_mode") {

			@Override
			public void initialize() {
				cargoState = true;
				hatchState = false;
			}

			@Override
			public void execute() {

			}

			@Override
			public boolean isFinished() {
				return true;
			}

			@Override
			public void end() {
				
			}
		};

		new SubsystemCommand(this.registeredCommands, "set_hatch_mode") {

			@Override
			public void initialize() {
				cargoState = false;
				hatchState = true;
			}

			@Override
			public void execute() {

			}

			@Override
			public boolean isFinished() {
				return true;
			}

			@Override
			public void end() {
				
			}
		};
    }

	@Override
	public void init() {
		cargoMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		hatchplatePump.setIdleMode(CANSparkMax.IdleMode.kBrake);

		cargoMotor.set(0);
		hatchplatePump.set(0);
	}

	@Override
	public void destruct() {
		cargoMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		hatchplatePump.setIdleMode(CANSparkMax.IdleMode.kBrake);

		cargoMotor.set(0);
		hatchplatePump.set(0);
	}
}