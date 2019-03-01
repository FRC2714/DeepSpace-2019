package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import frc.robot.util.ControlsProcessor;
import frc.robot.util.SubsystemCommand;
import frc.robot.util.SubsystemModule;

public class Arm extends SubsystemModule {

	private Intake intake;

	// Arm motors
	private final CANSparkMax shoulderMotor = new CANSparkMax(7, MotorType.kBrushless);
	private final CANSparkMax wristMotor = new CANSparkMax(8, MotorType.kBrushless);

	// Initialize arm encoders
	private CANEncoder shoulderMotorEncoder = shoulderMotor.getEncoder();
	private CANEncoder wristEncoder = wristMotor.getEncoder();

	// Arm characteristics
	private final double wristRatio = 360 / (140.0);
	private final double shoulderRatio = 360 / (512.0 / 3);

	// Current arm angles in degrees
	private double currentShoulderAngle;
	private double currentWristAngle;

	// Desired arm angles in degrees
	private double desiredShoulderAngle;
	private double desiredWristAngle;

	// Start arm anngles
	private double startShoulderAngle;
	private double startWristAngle;

	// Movement direction
	private double shoulderCoefficient;
	private double wristCoefficient;

	// Arm start powers
	private double shoulderPowerFactor;
	private double wristPowerFactor;

	// Arm initialization
	public Arm(ControlsProcessor controlsProcessor) {
		intake = new Intake();

		controlsProcessor.registerController("Intake", intake);
		registerCommands();

		// Enable voltage compensation for arm motors
		shoulderMotor.enableVoltageCompensation(12.0);
		wristMotor.enableVoltageCompensation(12.0);

		// Set SparkMax CAN periods
		shoulderMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
		shoulderMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 5);
		wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
		wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);

		// Inverts wrist motor direction
		wristMotor.setInverted(true);
	}

	/**
	 * Sets currentShoulderAngle and currentWristAngle to the encoder values
	 */
	public void updateCurrentAngles() {
		currentShoulderAngle = shoulderMotorEncoder.getPosition() * shoulderRatio;
		currentWristAngle = wristEncoder.getPosition() * wristRatio;
	}

	/**
	 * Uses costrol and feedforward value to set shoulder motor power
	 */
	public void setShoulderPower() {
		double motorPower = 0;

		if(Math.abs(desiredShoulderAngle - currentShoulderAngle) < 0.5) {}
		else if(desiredShoulderAngle - currentShoulderAngle != 0) {
			motorPower += Math.pow(desiredShoulderAngle - currentShoulderAngle, 2);
			motorPower *= shoulderCoefficient;
			motorPower = Math.sin(motorPower);
			motorPower += 0.2;
			motorPower *= (int)((desiredShoulderAngle - currentShoulderAngle) / Math.abs(desiredShoulderAngle - currentShoulderAngle));
			motorPower *= shoulderPowerFactor;
		}

		shoulderMotor.set(motorPower);
	}

	/**
	 * Plugs in desired wrist angle to the wrist costrol
	 */
	public void setWristPower() {
		double motorPower = 0;

		if(Math.abs(desiredWristAngle - currentWristAngle) < 0.5) {}
		else if(desiredWristAngle - currentWristAngle != 0) {
			motorPower += Math.pow(desiredWristAngle - currentWristAngle, 2);
			motorPower *= wristCoefficient;
			motorPower = Math.sin(motorPower);
			motorPower += 0.05;
			motorPower *= (int)((desiredWristAngle - currentWristAngle) / Math.abs(desiredWristAngle - currentWristAngle));
			motorPower *= wristPowerFactor;
		}

		wristMotor.set(motorPower);
	}

	/**
	 *
	 * @param shoulderPower
	 * @param wristPower
	 */
	public void setPowerFactors(double shoulderPower, double wristPower) {
		shoulderPowerFactor = (Math.abs(desiredShoulderAngle - startShoulderAngle) / 40) * shoulderPower;
		if(shoulderPowerFactor > shoulderPower) { shoulderPowerFactor = shoulderPower; }

		wristPowerFactor = (Math.abs(desiredWristAngle - startWristAngle) / 40) * wristPower;
		if(wristPowerFactor > wristPower) { wristPowerFactor = wristPower; }
	}

	/**
	 *
	 */
	public void setArmValues() {
		if(desiredShoulderAngle - startShoulderAngle != 0) {
			shoulderCoefficient = Math.PI / (2 * Math.pow(desiredShoulderAngle - startShoulderAngle, 2));
		}
		else {
			shoulderCoefficient = 0;
		}

		if(desiredWristAngle - startWristAngle != 0) {
			wristCoefficient = Math.PI / (2 * Math.pow(desiredWristAngle - startWristAngle, 2));
		}
		else {
			wristCoefficient = 0;
		}
	}

	/**
	 * Sets the arm to the desired overall position
	 * @param shoulderAngle the desired shoulder angle in degrees
	 * @param wristAngle the desired wrist angle in degrees
	 */
	public void goToPosition(double shoulderAngle, double wristAngle) {
		desiredShoulderAngle = shoulderAngle;
		desiredWristAngle = wristAngle;

		startShoulderAngle = currentShoulderAngle;
		startWristAngle = currentWristAngle;

		setPowerFactors(0.75, 1.0);
		setArmValues();
	}

	/**
	 *
	 * @param shoulderAngle
	 * @param wristAngle
	 * @return
	 */
	public boolean atPosition(double shoulderAngle, double wristAngle) {
		boolean atShoulderAngle = false;
		boolean atWristAngle = false;

		double shoulderAngleDelta = Math.abs(currentShoulderAngle - shoulderAngle);
		double wristAngleDelta = Math.abs(currentWristAngle - wristAngle);

		if(shoulderAngleDelta < 2.0) { atShoulderAngle = true; }
		if(wristAngleDelta < 2.0) { atWristAngle = true; }

		if(atShoulderAngle && atWristAngle) {
			intake.setAtPosition(true);
			return true;
		} else {
			intake.setAtPosition(false);
			return false;
		}
	}

	@Override
	public void run() {
		updateCurrentAngles();

		setShoulderPower();
		setWristPower();

		if(atPosition(desiredShoulderAngle, desiredWristAngle)) { intake.setAtPosition(true); }
	}

	@Override
	public void registerCommands() {
		new SubsystemCommand(this.registeredCommands, "jog_up") {
			@Override
			public void initialize() {}

			@Override
			public void execute() {
				goToPosition(currentShoulderAngle + 3, currentWristAngle + 1.5);
			}

			@Override
			public boolean isFinished() {
				return false;
			}

			@Override
			public void end() {}
		};

		new SubsystemCommand(this.registeredCommands, "jog_down") {
			@Override
			public void initialize() {}

			@Override
			public void execute() {
				goToPosition(currentShoulderAngle - 3, currentWristAngle - 3);
			}

			@Override
			public boolean isFinished() {
				return false;
			}

			@Override
			public void end() {}
		};

		new SubsystemCommand(this.registeredCommands, "arm_to_position") {
			double shoulderAngle;
			double wristAngle;

			@Override
			public void initialize() {
				shoulderAngle = Double.parseDouble(this.args[0]);
				wristAngle = Double.parseDouble(this.args[1]);

				goToPosition(shoulderAngle, wristAngle);
			}

			@Override
			public void execute() {}

			@Override
			public boolean isFinished() {
				return atPosition(shoulderAngle, wristAngle);
			}

			@Override
			public void end() {}
		};

		new SubsystemCommand(this.registeredCommands, "floor_cargo_position") {
			double shoulderAngle;
			double wristAngle;

			@Override
			public void initialize() {
				shoulderAngle = 27;
				wristAngle = 240;

				goToPosition(shoulderAngle, wristAngle);
			}

			@Override
			public void execute() {}

			@Override
			public boolean isFinished() {
				return atPosition(shoulderAngle, wristAngle);
			}

			@Override
			public void end() {}
		};

		new SubsystemCommand(this.registeredCommands, "floor_hatch_position") {
			double shoulderAngle;
			double wristAngle;

			@Override
			public void initialize() {
				shoulderAngle = 19;
				wristAngle = 193;

				goToPosition(shoulderAngle, wristAngle);
			}

			@Override
			public void execute() {}

			@Override
			public boolean isFinished() {
				return atPosition(shoulderAngle, wristAngle);
			}

			@Override
			public void end() {}
		};



		new SubsystemCommand(this.registeredCommands, "station_position") {
			double shoulderAngle;
			double wristAngle;

			@Override
			public void initialize() {
				shoulderAngle = 4;
				wristAngle = 80;

				goToPosition(shoulderAngle, wristAngle);
			}

			@Override
			public void execute() {}

			@Override
			public boolean isFinished() {
				return atPosition(shoulderAngle, wristAngle);
			}

			@Override
			public void end() {}
		};

		new SubsystemCommand(this.registeredCommands, "lower_score") {
			double shoulderAngle;
			double wristAngle;

			@Override
			public void initialize() {
				if(intake.getCargoState()) {
					shoulderAngle = 28;
					wristAngle = 95;
				} else if(intake.getHatchState()) {
					shoulderAngle = 4;
					wristAngle = 82;
				} else {
					shoulderAngle = currentShoulderAngle;
					wristAngle = currentWristAngle;
				}

				goToPosition(shoulderAngle, wristAngle);
			}

			@Override
			public void execute() {}

			@Override
			public boolean isFinished() {
				return atPosition(shoulderAngle, wristAngle);
			}

			@Override
			public void end() {}
		};

		new SubsystemCommand(this.registeredCommands, "middle_score") {
			double shoulderAngle;
			double wristAngle;

			@Override
			public void initialize() {
				if(intake.getCargoState()) {
					shoulderAngle = 95;
					wristAngle = 200;
				} else if(intake.getHatchState()) {
					shoulderAngle = 66;
					wristAngle = 129;
				} else {
					shoulderAngle = currentShoulderAngle;
					wristAngle = currentWristAngle;
				}

				goToPosition(shoulderAngle, wristAngle);
			}

			@Override
			public void execute() {}

			@Override
			public boolean isFinished() {
				return atPosition(shoulderAngle, wristAngle);
			}

			@Override
			public void end() {}
		};

		new SubsystemCommand(this.registeredCommands, "upper_score") {
			double shoulderAngle;
			double wristAngle;

			@Override
			public void initialize() {
				if(intake.getCargoState()) {
					shoulderAngle = 122;
					wristAngle = 217;
				} else if(intake.getHatchState()) {
					shoulderAngle = 116;
					wristAngle = 185;
				} else {
					shoulderAngle = currentShoulderAngle;
					wristAngle = currentWristAngle;
				}

				goToPosition(shoulderAngle, wristAngle);
			}

			@Override
			public void execute() {}

			@Override
			public boolean isFinished() {
				return atPosition(shoulderAngle, wristAngle);
			}

			@Override
			public void end() {}
		};

		new SubsystemCommand(this.registeredCommands, "back_score") {
			double shoulderAngle;
			double wristAngle;

			@Override
			public void initialize() {
				if(intake.getHatchState()) {
					shoulderAngle = 130;
					wristAngle = 58;
				} else {
					shoulderAngle = currentShoulderAngle;
					wristAngle = currentWristAngle;
				}

				goToPosition(shoulderAngle, wristAngle);
			}

			@Override
			public void execute() {}

			@Override
			public boolean isFinished() {
				return atPosition(shoulderAngle, wristAngle);
			}

			@Override
			public void end() {}
		};

		new SubsystemCommand(this.registeredCommands, "extake") {

			int timer = 0;

			@Override
			public void initialize() {
				if(intake.getHatchState()) {
					intake.pumpMotor.set(0);
				} else {
					intake.cargoMotor.set(-1);
				}
				intake.clearStates();
			}

			@Override
			public void execute() {
				intake.pumpRelease();
				timer += 20;
			}

			@Override
			public boolean isFinished() {
				return timer >= 500;
			}

			@Override
			public void end() {
				intake.cargoMotor.set(0);
			}
		};
	}

	@Override
	public void init() {
		intake.init();

		shoulderMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		wristMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

		shoulderMotorEncoder.setPosition(0);
		wristEncoder.setPosition(0);

		desiredShoulderAngle = 0;
		desiredWristAngle = 0;

		startShoulderAngle = 0;
		startWristAngle = 0;
	}

	@Override
	public void destruct() {
		intake.destruct();

		shoulderMotor.set(0);
		wristMotor.set(0);

		shoulderMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		wristMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

		System.out.println("Current Shoulder Angle: " + currentShoulderAngle + "\tCurrent Wrist Angle: " + currentWristAngle);
	}
}