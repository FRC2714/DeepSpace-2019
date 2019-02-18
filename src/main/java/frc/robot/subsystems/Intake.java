package frc.robot.subsystems;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.util.SubsystemCommand;
import frc.robot.util.SubsystemModule;

public class Intake extends SubsystemModule {

	// Intake Motors
	protected CANSparkMax cargoMotor = new CANSparkMax(9, MotorType.kBrushless);
	private CANSparkMax pumpMotor = new CANSparkMax(10, MotorType.kBrushless);

	// Hatchplate Servo
	private Servo hatchplateServo = new Servo(0);
	private Servo valveServo1 = new Servo(1);
	private Servo valveServo2 = new Servo(2);

	// Maximum currents for cargo and hatch intakes
	private final double cargoCurrentThreshold = 30;
	private final double hatchCurrentThreshold = 10;
	private final double pumpCurrentDiffrence = 1; // 3.125 for two

	// Pump state values
	private double pumpStateFirstAvg;
	private boolean pumpStateIsFirstAvg;
	private int pumpStateCounter;

	// ArrayList holding the read currents
	private ArrayList<Double> cargoCurrents;
	private ArrayList<Double> hatchCurrents;
	private ArrayList<Double> pumpCurrents;

	// Average current
	private double cargoAverageCurrent;
	private double hatchAverageCurrent;
	private double pumpAverageCurrent;

	// Number of current values stored
	private final int numberOfCargoCurrents = 50;
	private final int numberOfHatchCurrents = 50;
	private final int numberOfPumpCurrents = 100;

	// Intake States - Public so Arm can access the states for state-based logic
	private boolean cargoState;
	private boolean hatchState;
	private boolean pumpState;

	// Intake position
	private boolean atPosition;

    public Intake() { 
		registerCommands(); // Puts commands onto the hashmap
	}

	/**
	 * Raises the hatchplate into tucked position
	 */
	public void hatchplateIn() {
		hatchplateServo.set(1);
	}

	/**
	 * Lowers the hatchplate into active position
	 */
	public void hatchplateOut() {
		hatchplateServo.set(0);
	}

	/**
	 * Puts pump servo into hatch intake mode
	 */
	public void pumpHatch() {
		valveServo1.set(0.64);
		valveServo2.set(0.11);
	}

	/**
	 * Puts pump servo into release mode
	 */
	public void pumpRelease() {
		valveServo1.set(0.55);
		valveServo2.set(0.64);
	}

	/**
	 * Puts pump servo into climb mode
	 */
	public void pumpClimb() {
		valveServo1.set(0);
		valveServo2.set(0);
	}

	/**
	 * Uses the roller's current draw to determine if the robot has a cargo
	 * @return Cargo = true or 
	 */
	public boolean checkCargoState() {
		if(cargoMotor.get() > 0.5) {
			cargoCurrents.add(0, Math.abs(cargoMotor.getOutputCurrent()));

			if(cargoCurrents.size() != numberOfCargoCurrents) {
				cargoAverageCurrent += cargoCurrents.get(0) / numberOfCargoCurrents;

				return cargoState;
			}
			else {
				cargoAverageCurrent += (cargoCurrents.get(0) - cargoCurrents.get(cargoCurrents.size() - 1)) / numberOfCargoCurrents;
				cargoCurrents.remove(cargoCurrents.size() - 1);

				return cargoAverageCurrent > cargoCurrentThreshold;
			}
		}

		// cargoCurrents.clear();
		cargoCurrents = new ArrayList<Double>(0);
		cargoAverageCurrent = 0;

		return cargoState;
	}

	public void clearStates() {
		cargoState = false;
		hatchState = false;
		pumpState = false;
	}

	/**
	 * Compares the first average of currents with all subsequent averages to determine if the robot has a hatch
	 * @return if the robot has a hatch
	 */
	public boolean checkHatchState() {
		if(cargoMotor.get() < -0.5) {
			hatchCurrents.add(0, Math.abs(cargoMotor.getOutputCurrent()));

			if(hatchCurrents.size() != numberOfHatchCurrents) {
				hatchAverageCurrent += hatchCurrents.get(0) / numberOfHatchCurrents;

				return hatchState;
			}
			else {
				hatchAverageCurrent += (hatchCurrents.get(0) - hatchCurrents.get(hatchCurrents.size() - 1)) / numberOfHatchCurrents;
				hatchCurrents.remove(hatchCurrents.size() - 1);

				return Math.abs(hatchAverageCurrent) > hatchCurrentThreshold;
			}
		}

		// hatchCurrents.clear();
		hatchCurrents = new ArrayList<Double>(0);
		hatchAverageCurrent = 0;

		return hatchState;
	}

	public boolean checkPumpState() {
		if(pumpMotor.get() != 0) {
			pumpCurrents.add(0, Math.abs(pumpMotor.getOutputCurrent()));

			if(pumpCurrents.size() != numberOfPumpCurrents) {
				pumpAverageCurrent += pumpCurrents.get(0) / numberOfPumpCurrents;
				pumpStateCounter = 0;
				return pumpState;
			}
			else {
				if(pumpStateCounter > 10 && pumpStateIsFirstAvg) {
					pumpStateFirstAvg = pumpAverageCurrent;
					System.out.println("First CD: " + pumpStateFirstAvg);
					pumpStateIsFirstAvg = false;
				}

				pumpStateCounter++;

				System.out.println("Current: " + pumpAverageCurrent);
				pumpAverageCurrent += (pumpCurrents.get(0) - pumpCurrents.get(pumpCurrents.size() - 1)) / numberOfPumpCurrents;
				pumpCurrents.remove(pumpCurrents.size() - 1);

				return Math.abs(pumpAverageCurrent) < Math.abs(pumpStateFirstAvg) - pumpCurrentDiffrence;
			}
		}

		// pumpCurrents.clear();
		pumpCurrents = new ArrayList<Double>(0);
		pumpAverageCurrent = 0;
		pumpStateFirstAvg = 0;
		pumpStateCounter = 0;
		pumpStateIsFirstAvg = true;

		return pumpState;
	}

	public boolean getCargoState() {
		return cargoState;
	}

	public boolean getHatchState() {
		return pumpState;
	}

	public void setAtPosition(boolean atPosition) {
		this.atPosition = atPosition;
	}

	@Override
	public void run() {
		cargoState = checkCargoState();
		hatchState = checkHatchState();
		pumpState = checkPumpState();

		// if(cargoCurrents.size() >= numberOfCargoCurrents || hatchCurrents.size() >= numberOfHatchCurrents || pumpCurrents.size() >= numberOfPumpCurrents)
		// 	System.out.println("Cargo Current: " + cargoAverageCurrent + "\tHatch Current: " + hatchAverageCurrent + "\tPump Current: " + pumpAverageCurrent);
    }

	@Override
	public void registerCommands() {

		new SubsystemCommand(this.registeredCommands, "cargo_true") {

			@Override
			public void initialize() {
				cargoState = true;
				hatchState = false;
				pumpState = false;

				System.out.println("Override cargo");
			}

			@Override
			public void execute() {}

			@Override
			public boolean isFinished() {
				return true;
			}

			@Override
			public void end() {}
		};

		new SubsystemCommand(this.registeredCommands, "hatch_true") {

			@Override
			public void initialize() {
				cargoState = false;
				hatchState = false;
				pumpState = true;
				System.out.println("Override hatch station");
			}

			@Override
			public void execute() {}

			@Override
			public boolean isFinished() {
				return true;
			}

			@Override
			public void end() {}
		};

		new SubsystemCommand(this.registeredCommands, "hatch_floor_true") {

			@Override
			public void initialize() {
				cargoState = false;
				hatchState = false;
				pumpState = true;
				System.out.println("Override hatch floor");
			}

			@Override
			public void execute() {}

			@Override
			public boolean isFinished() {
				return true;
			}

			@Override
			public void end() {}
		};
		
		new SubsystemCommand(this.registeredCommands, "intake_stop") {

			@Override
			public void initialize() {
				cargoMotor.set(0);
				pumpMotor.set(0);

				cargoState = false;
				hatchState = false;
				pumpState = false;
			}

			@Override
			public void execute() {
				pumpRelease();
			}

			@Override
			public boolean isFinished() {
				return false;
			}

			@Override
			public void end() {}
		};


        new SubsystemCommand(this.registeredCommands, "cargo_intake") {
			boolean intaking;

			@Override
			public void initialize() {
				intaking = false;
			}

			@Override
			public void execute() {
				hatchplateIn();

				if (!intaking && atPosition) {
					cargoMotor.set(0.75);
					intaking = true;
				}
			}

			@Override
			public boolean isFinished() {
				return cargoState || hatchState || pumpState;
			}

			@Override
			public void end() {
				cargoMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

				if (cargoState) {
					cargoMotor.set(0.05);
				} else {
					cargoMotor.set(0);
				}
			}
		};

        new SubsystemCommand(this.registeredCommands, "hatch_floor_intake") {
			boolean intaking;

			@Override
			public void initialize() {
				intaking = false;
			}

			@Override
			public void execute() {
				pumpHatch();

				if (!intaking && atPosition) {
					cargoMotor.set(-0.75);
					intaking = true;
				}
				else if (hatchState) {
					pumpMotor.set(1);
					hatchplateOut();
				}
				else {
					hatchplateIn();
				}
			}

			@Override
			public boolean isFinished() {
				return cargoState || pumpState;
			}

			@Override
			public void end() {
				pumpMotor.set(0);

				if(!cargoState) { cargoMotor.set(0); }
				else if(pumpState) { hatchState = false; }
			}
		};

        new SubsystemCommand(this.registeredCommands, "hatch_station_intake") {
			boolean intaking;

			@Override
			public void initialize() {
				intaking = false;
			}

			@Override
			public void execute() {
				pumpHatch();
				hatchplateOut();

				if(!intaking && atPosition) {
					pumpMotor.set(1);
					intaking = true;
				}
			}

			@Override
			public boolean isFinished() {
				return cargoState || hatchState || pumpState;
			}

			@Override
			public void end() {
				cargoMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
				pumpMotor.set(0);
			}
		};

		new SubsystemCommand(this.registeredCommands, "set_cargo_mode") {

			@Override
			public void initialize() {
				cargoState = true;
				hatchState = false;
			}

			@Override
			public void execute() { }

			@Override
			public boolean isFinished() {
				return true;
			}

			@Override
			public void end() { }
		};

		new SubsystemCommand(this.registeredCommands, "set_hatch_mode") {

			@Override
			public void initialize() {
				cargoState = false;
				hatchState = true;
			}

			@Override
			public void execute() { }

			@Override
			public boolean isFinished() {
				return true;
			}

			@Override
			public void end() { }
		};
    }

	@Override
	public void init() {
		cargoMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		pumpMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

		cargoMotor.set(0);
		pumpMotor.set(0);

		cargoCurrents = new ArrayList<Double>(0);
		hatchCurrents = new ArrayList<Double>(0);
		pumpCurrents = new ArrayList<Double>(0);

		cargoAverageCurrent = 0;
		hatchAverageCurrent = 0;
		pumpAverageCurrent = 0;

		cargoState = false;
		hatchState = false;
		pumpState = false;

		atPosition = false;
		pumpStateIsFirstAvg = true;
	}

	@Override
	public void destruct() {
		cargoMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		pumpMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

		cargoMotor.set(0);
		pumpMotor.set(0);
	}
}