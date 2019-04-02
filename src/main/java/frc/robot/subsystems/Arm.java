package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
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
	private CANEncoder shoulderEncoder = shoulderMotor.getEncoder();
	private CANEncoder wristEncoder = wristMotor.getEncoder();

	// Arm PIDs
	private CANPIDController shoulderPID;
	private CANPIDController wristPID;

	// ControlsProcessor
	private ControlsProcessor controlsProcessor;

	// Arm initialization
	public Arm(ControlsProcessor controlsProcessor) {
		intake = new Intake();

		controlsProcessor.registerController("Intake", intake);
		this.controlsProcessor = controlsProcessor;
		registerCommands();

		// Enable voltage compensation for arm motors
		shoulderMotor.enableVoltageCompensation(12.0);
		wristMotor.enableVoltageCompensation(12.0);

		shoulderMotor.setSmartCurrentLimit(80);
		wristMotor.setSmartCurrentLimit(80);

		// Set SparkMax CAN periods
		shoulderMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
		shoulderMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 5);
		wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
		wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);

		// Inverts wrist motor direction
		wristMotor.setInverted(true);

		// Setups arm PIDs
		shoulderPID = shoulderMotor.getPIDController();
		wristPID = wristMotor.getPIDController();

		shoulderPID.setP(0.6);
		shoulderPID.setI(0.00007);

		wristPID.setP(0.1);

		// Converts motor rotations to degrees
		shoulderEncoder.setPositionConversionFactor(1.0/7);
		wristEncoder.setPositionConversionFactor(18.0/7);
	}

	/**
	 * Sets the arm to the desired overall position
	 * @param leadscrewLength the desired shoulder angle in degrees
	 * @param wristAngle the desired wrist angle in degrees
	 */
	public void goToPosition(double leadscrewLength, double wristAngle) {
		shoulderPID.setReference(leadscrewLength, ControlType.kPosition);
		wristPID.setReference(wristAngle, ControlType.kPosition);
	}

	public boolean atPosition(double leadscrewLength) {
		return Math.abs(shoulderEncoder.getPosition() - leadscrewLength) < 0.1;
	}

	@Override
	public void run() {}

	@Override
	public void registerCommands() {
		new SubsystemCommand(this.registeredCommands, "jog_up") {
			double position;

			@Override
			public void initialize() {
				position = shoulderEncoder.getPosition();
			}

			@Override
			public void execute() {
				position += 0.05;
				goToPosition(position, wristEncoder.getPosition());
			}

			@Override
			public boolean isFinished() {
				return false;
			}

			@Override
			public void end() {}
		};

		new SubsystemCommand(this.registeredCommands, "jog_down") {
			double position;

			@Override
			public void initialize() {
				position = shoulderEncoder.getPosition();
			}

			@Override
			public void execute() {
				position -= 0.05;
				goToPosition(position, wristEncoder.getPosition());
			}

			@Override
			public boolean isFinished() {
				return false;
			}

			@Override
			public void end() {}
		};

		new SubsystemCommand(this.registeredCommands, "start_position") {
			double shoulderAngle;
			double wristAngle;

			@Override
			public void initialize() {
				shoulderAngle = 0;
				wristAngle = 0;

				goToPosition(shoulderAngle, wristAngle);
			}

			@Override
			public void execute() {}

			@Override
			public boolean isFinished() {
				return atPosition(shoulderAngle);
			}

			@Override
			public void end() {}
		};

		new SubsystemCommand(this.registeredCommands, "floor_cargo_position") {
			double shoulderAngle;
			double wristAngle;

			@Override
			public void initialize() {
				shoulderAngle = 2.4;
				wristAngle = 240;

				goToPosition(shoulderAngle, wristAngle);
			}

			@Override
			public void execute() {}

			@Override
			public boolean isFinished() {
				return atPosition(shoulderAngle);
			}

			@Override
			public void end() {}
		};

		new SubsystemCommand(this.registeredCommands, "floor_hatch_position") {
			double shoulderAngle;
			double wristAngle;

			@Override
			public void initialize() {
				shoulderAngle = 1.8;
				wristAngle = 195;

				goToPosition(shoulderAngle, wristAngle);
			}

			@Override
			public void execute() {}

			@Override
			public boolean isFinished() {
				return atPosition(shoulderAngle);
			}

			@Override
			public void end() {}
		};

		new SubsystemCommand(this.registeredCommands, "station_position") {
			double shoulderAngle;
			double wristAngle;

			@Override
			public void initialize() {
				shoulderAngle = 0.8;
				wristAngle = 86;

				goToPosition(shoulderAngle, wristAngle);
			}

			@Override
			public void execute() {}

			@Override
			public boolean isFinished() {
				return atPosition(shoulderAngle);
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
					shoulderAngle = 6;
					wristAngle = 205;
				} else {
					shoulderAngle = 0.6;
					wristAngle = 86;
				}

				goToPosition(shoulderAngle, wristAngle);
			}

			@Override
			public void execute() {}

			@Override
			public boolean isFinished() {
				return atPosition(shoulderAngle);
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
					shoulderAngle = 10.5;
					wristAngle = 230;
				} else {
					shoulderAngle = 6.8;
					wristAngle = 133;
				}

				goToPosition(shoulderAngle, wristAngle);
			}

			@Override
			public void execute() {}

			@Override
			public boolean isFinished() {
				return atPosition(shoulderAngle);
			}

			@Override
			public void end() {}
		};

		new SubsystemCommand(this.registeredCommands, "upper_score") {
			double shoulderAngle;
			double wristAngle;
			long startTime;

			@Override
			public void initialize() {
				if(intake.getCargoState()) {
					shoulderAngle = 13.4;
					wristAngle = 135;
				} else {
					shoulderAngle = 11.6;
					wristAngle = 192;
				}

				goToPosition(shoulderAngle, wristAngle);
				startTime = System.nanoTime();
			}

			@Override
			public void execute() {}

			@Override
			public boolean isFinished() {
				return System.nanoTime() - startTime > 1e9;
			}

			@Override
			public void end() {}
		};

		new SubsystemCommand(this.registeredCommands, "flex_score") {
			double shoulderAngle;
			double wristAngle;
			long startTime;

			@Override
			public void initialize() {

				if(intake.getCargoState()) {
					shoulderAngle = 9;
					wristAngle = 285;
				} else {
					shoulderAngle = 13.4;
					wristAngle = 65;
				}

				goToPosition(shoulderAngle, wristAngle);
				startTime = System.nanoTime();
			}

			@Override
			public void execute() {}

			@Override
			public boolean isFinished() {
				return System.nanoTime() - startTime > 1e9;
			}

			@Override
			public void end() {}
		};

		new SubsystemCommand(this.registeredCommands, "delayed_to_position") {
			double shoulderAngle;
			double wristAngle;

			int currentPeriod;
			int finalPeriod;

			@Override
			public void initialize() {
				shoulderAngle = Double.parseDouble(this.args[0]);
				wristAngle = Double.parseDouble(this.args[1]);

				currentPeriod = 0;
				finalPeriod = (int)(Double.parseDouble(this.args[2]) / controlsProcessor.getCommandPeriod());

				System.out.println("Final Period: " + finalPeriod);

			}

			@Override
			public void execute() {
				if(currentPeriod == finalPeriod) {
//					System.out.println("Current Period: " + currentPeriod);
					goToPosition(shoulderAngle, wristAngle);
					currentPeriod++;
				} else {
					currentPeriod++;
				}
			}

			@Override
			public boolean isFinished() {
				return atPosition(shoulderAngle);
			}

			@Override
			public void end() {}
		};

		new SubsystemCommand(this.registeredCommands, "auton_hatch") {
			double shoulderAngle;
			double wristAngle;

			@Override
			public void initialize() {
				shoulderAngle = 3;
				wristAngle = 110;

				goToPosition(shoulderAngle, wristAngle);
			}

			@Override
			public void execute() {}

			@Override
			public boolean isFinished() {
				return atPosition(shoulderAngle);
			}

			@Override
			public void end() {}
		};

		new SubsystemCommand(this.registeredCommands, "extake") {
			long timer;

			@Override
			public void initialize() {
				timer = System.nanoTime();

				intake.pumpMotor.set(0);

				if(!intake.getHatchState())
					intake.cargoMotor.set(-1);

				intake.clearStates();
			}

			@Override
			public void execute() {
				intake.pumpRelease();
			}

			@Override
			public boolean isFinished() {
				return System.nanoTime() - timer >= 0.75 * 1e9;
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
		wristMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

		shoulderEncoder.setPosition(0);
		wristEncoder.setPosition(0);
	}

	@Override
	public void destruct() {
		intake.destruct();

		shoulderMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		wristMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
	}
}