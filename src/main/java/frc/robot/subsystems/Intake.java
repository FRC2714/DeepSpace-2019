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
	private Servo hatchplateServo0 = new Servo(0);
	private Servo hatchplateServo1 = new Servo(1);

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

    public Intake() { 
		registerCommands(); // Puts commands onto the hashmap
	}

	/**
	 * Raises the hatchplate into tucked position
	 */
	public void hatchplateUp() {
		hatchplateServo0.set(1);
		hatchplateServo1.set(1);
	}

	/**
	 * Lowers the hatchplate into active position
	 */
	public void hatchplateDown() {
		hatchplateServo0.set(0.47);
		hatchplateServo1.set(0.47);
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

	@Override
	public void run() {
    }

	@Override
	public void registerCommands() {

        new SubsystemCommand(this.registeredCommands, "cargo_intake") {

			boolean stopIntake = false;

			@Override
			public void initialize() {
				if (!cargoState && !hatchState) {
					cargoMotor.set(0.75);
				} else {
					stopIntake = true;
				}
			}

			@Override
			public void execute() {
				if (cargoMotor.getOutputCurrent() > cargoCurrentThreshold) { cargoState = true; }
			}

			@Override
			public boolean isFinished() {
				return cargoState || stopIntake;
			}

			@Override
			public void end() {
				cargoMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

				if (cargoState && !hatchState) {
					cargoMotor.set(0.05);
				} else {
					cargoMotor.set(0);
				}
			}
		};

        new SubsystemCommand(this.registeredCommands, "hatch_floor_intake") {

			boolean stopIntake = false;

			@Override
			public void initialize() {
				if (!cargoState && !hatchState) {
					cargoMotor.set(-0.75);
				} else {
					stopIntake = true;
				}
			}

			@Override
			public void execute() {
				if (cargoMotor.getOutputCurrent() < hatchCurrentThreshold) { hatchState = true; }
			}

			@Override
			public boolean isFinished() {
				return hatchState || stopIntake;
			}

			@Override
			public void end() {
				cargoMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
				cargoMotor.set(0);

				if (!cargoState && hatchState) {
					hatchplatePump.set(1);
					hatchplateDown();

					hatchFloor = true;
					hatchStation = false;
				}
			}
		};

        new SubsystemCommand(this.registeredCommands, "hatch_station_intake") {
			
			ArrayList<Double> currents = new ArrayList<Double>();
			double avgCurrent = 0;
			double maxSize = 50;

			@Override
			public void initialize() {
				if (!cargoState && !hatchState) {
					hatchplatePump.set(1);
					hatchplateDown();
				}
			}

			@Override
			public void execute() {
				currents.add(0, hatchplatePump.getOutputCurrent());
				
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
				return hatchState;
			}

			@Override
			public void end() {
				cargoMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

				if (!cargoState && hatchState) {
					hatchFloor = false;
					hatchStation = true;
				}
			}
		};

        new SubsystemCommand(this.registeredCommands, "extake") {

			@Override
			public void initialize() {
				if (cargoState && !hatchState) {
					cargoMotor.set(-0.5);
				} else if (!cargoState && hatchState) {
					//TODO: Open the blowout valve
					hatchplatePump.set(0);
				}
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
				cargoMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

				hatchplateUp();
				cargoMotor.set(0);
				//TODO: Close the blowout valve

				cargoState = false;
				hatchState = false;

				hatchFloor = false;
				hatchStation = false;
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