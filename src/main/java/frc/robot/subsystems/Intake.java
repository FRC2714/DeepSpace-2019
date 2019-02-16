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
	private CANSparkMax pumpMotor = new CANSparkMax(10, MotorType.kBrushless);

	// Hatchplate Servo
	private Servo hatchplateServo = new Servo(0);
	private Servo valveServo1 = new Servo(1);
	private Servo valveServo2 = new Servo(2);

	// Maximum currents for cargo and hatch intakes
	private double cargoCurrentThreshold = 30;
	private double hatchCurrentThreshold = 100;
	private double pumpCurrentThreshold = 5.5;

	// ArrayList holding the read currents
	private ArrayList<Double> cargoCurrents;
	private ArrayList<Double> hatchCurrents;
	private ArrayList<Double> pumpCurrents;

	// Average current
	private double cargoAverageCurrent = 0;
	private double hatchAverageCurrent = 0;
	private double pumpAverageCurrent = 0;

	// Number of current values stored
	private int numberOfCargoCurrents = 50;
	private int numberOfHatchCurrents = 50;
	private int numberOfPumpCurrents = 100;

	// Intake States - Public so Arm can access the states for state-based logic
	private boolean cargoState = false;
	private boolean hatchState = false;
	private boolean pumpState = false;

	// Hatch intake types
	private boolean hatchFloor = false;
	private boolean hatchStation = false;

	// Intake position
	private boolean atPosition = false;

    public Intake() { 
		registerCommands(); // Puts commands onto the hashmap
		cargoCurrents = new ArrayList<Double>(0);
		hatchCurrents = new ArrayList<Double>(0);
		pumpCurrents = new ArrayList<Double>(0);
	}

	/**
	 * Raises the hatchplate into tucked position
	 */
	public void hatchplateUp() {
		hatchplateServo.set(0.88);
	}

	/**
	 * Lowers the hatchplate into active position
	 */
	public void hatchplateDown() {
		hatchplateServo.set(0.3);
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

	public boolean checkCargoState() {
		if(cargoMotor.get() == 0.75) {
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
		cargoCurrents.clear();
		cargoAverageCurrent = 0;
		return cargoState;
	}

	public boolean checkHatchState() {
		if(cargoMotor.get() == -0.75) {
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
		hatchCurrents.clear();
		hatchAverageCurrent = 0;
		return hatchState;
	}

	public boolean checkPumpState() {
		if(pumpMotor.get() == 1) {
			pumpCurrents.add(0, Math.abs(pumpMotor.getOutputCurrent()));
			if(pumpCurrents.size() != numberOfPumpCurrents) {
				pumpAverageCurrent += pumpCurrents.get(0) / numberOfPumpCurrents;
				return pumpState;
			}
			else {
				pumpAverageCurrent += (pumpCurrents.get(0) - pumpCurrents.get(pumpCurrents.size() - 1)) / numberOfPumpCurrents;
				pumpCurrents.remove(pumpCurrents.size() - 1);
				return Math.abs(pumpAverageCurrent) < pumpCurrentThreshold;
			}
		}
		pumpCurrents.clear();
		pumpAverageCurrent = 0;
		return pumpState;
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
	}

	@Override
	public void run() {
		cargoState = checkCargoState();
		hatchState = checkHatchState();
		pumpState = checkPumpState();

		if(cargoAverageCurrent != 0 || hatchAverageCurrent != 0 || pumpAverageCurrent != 0)
			System.out.println("Cargo Current: " + cargoAverageCurrent + "\tHatch Current: " + hatchAverageCurrent + "\tPump Current: " + pumpAverageCurrent);
    }

	@Override
	public void registerCommands() {

		new SubsystemCommand(this.registeredCommands, "intake_stop") {

			@Override
			public void initialize() {
				cargoMotor.set(0);
				pumpMotor.set(0);

				hatchState = false;
				cargoState = false;
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

		new SubsystemCommand(this.registeredCommands, "get_intake_servos") {

			@Override
			public void initialize() {
				System.out.println("Servo 1: " + valveServo1.get() + "\tServo 2: " + valveServo2.get());
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

		new SubsystemCommand(this.registeredCommands, "servo_sweep") {
			double servoPosition = 0;

			@Override
			public void initialize() {
				valveServo1.set(servoPosition);

				servoPosition += 0.02;
			}

			@Override
			public void execute() {}

			@Override
			public boolean isFinished() {
				return true;
			}

			@Override
			public void end() {
				if(servoPosition >= 1) { servoPosition = 0; }
				System.out.println("Sweep: " + servoPosition);
			}
		};

        new SubsystemCommand(this.registeredCommands, "cargo_intake") {
			boolean intaking;

			@Override
			public void initialize() {
				intaking = false;
			}

			@Override
			public void execute() {
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
				else if(hatchState) {
					pumpMotor.set(1);
					hatchplateDown();
				}
			}

			@Override
			public boolean isFinished() {
				return cargoState || pumpState;
			}

			@Override
			public void end() {
				cargoMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
				if(!cargoState) { cargoMotor.set(0); }
				pumpMotor.set(0);
				
				if (pumpState) {
					hatchState = false;

					hatchFloor = true;
					hatchStation = false;
				}
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

				if(!intaking && atPosition) {
					pumpMotor.set(1);
					hatchplateDown();
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

				if (pumpState) {
					hatchFloor = false;
					hatchStation = true;
				}
			}
		};

        new SubsystemCommand(this.registeredCommands, "extake") {

			@Override
			public void initialize() {
				if (!pumpState) {
					cargoMotor.set(-0.5);
				}
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
			public void end() {
				cargoMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

				hatchplateUp();
				cargoMotor.set(0);

				cargoState = false;
				hatchState = false;
				pumpState = false;

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
	}

	@Override
	public void destruct() {
		cargoMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		pumpMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

		cargoMotor.set(0);
		pumpMotor.set(0);
	}
}