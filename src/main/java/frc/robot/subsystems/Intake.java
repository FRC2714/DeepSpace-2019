package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.util.ControlsProcessor;
import frc.robot.util.SubsystemCommand;
import frc.robot.util.SubsystemModule;

public class Intake extends SubsystemModule {

	// Controls Processor
	private ControlsProcessor controlsProcessor;

	// Intake Motors
	private CANSparkMax cargoMotor = new CANSparkMax(9, MotorType.kBrushless);
	private CANSparkMax hatchplatePump = new CANSparkMax(10, MotorType.kBrushless);

	// Hatchplate Servo
	private Servo hatchplateServo0 = new Servo(0);
	private Servo hatchplateServo1 = new Servo(1);

	// Intake Sensors
	//TODO: These are placeholders
	private boolean cargoSensor = false;
	private boolean hatchSensor = false;

    public Intake(ControlsProcessor controlsProcessor) { 
		registerCommands(); // Puts commands onto the hashmap
		
		this.controlsProcessor = controlsProcessor;
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

	@Override
	public void run() {
		//TODO: Update intake sensors here
    }

	@Override
	public void registerCommands() {

        new SubsystemCommand(this.registeredCommands, "cargo_intake") {

			@Override
			public void initialize() {
				if (!cargoSensor && !hatchSensor) {
					cargoMotor.set(0.75);
				} else {
					end();
				}
			}

			@Override
			public void execute() {

			}

			@Override
			public boolean isFinished() {
				return cargoSensor;
			}

			@Override
			public void end() {
				cargoMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

				if (cargoSensor && !hatchSensor) {
					cargoMotor.set(0.05);
				}
			}
		};

        new SubsystemCommand(this.registeredCommands, "hatch_floor_intake") {

			@Override
			public void initialize() {
				if (!cargoSensor && !hatchSensor) {
					cargoMotor.set(-0.75);
				} else {
					end();
				}
			}

			@Override
			public void execute() {
			}

			@Override
			public boolean isFinished() {
				return hatchSensor;
			}

			@Override
			public void end() {
				cargoMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
				cargoMotor.set(0);

				if (!cargoSensor && hatchSensor) {
					hatchplatePump.set(1);
					hatchplateDown();
				}
			}
		};

        new SubsystemCommand(this.registeredCommands, "hatch_station_intake") {

			@Override
			public void initialize() {
				if (!cargoSensor && !hatchSensor) {
					hatchplatePump.set(1);
					hatchplateDown();
				}
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
				cargoMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
			}
		};

        new SubsystemCommand(this.registeredCommands, "extake") {

			@Override
			public void initialize() {
				if (cargoSensor && !hatchSensor) {
					cargoMotor.set(-0.5);
				} else if (!cargoSensor && hatchSensor) {
					hatchplatePump.set(0);
				} else {
					end();
				}
			}

			@Override
			public void execute() {

			}

			@Override
			public boolean isFinished() {
				return !hatchSensor && !cargoSensor;
			}

			@Override
			public void end() {
				cargoMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

				if (!cargoSensor && !hatchSensor) {
					hatchplateUp();
					cargoMotor.set(0);
				}
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