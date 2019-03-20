package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import frc.robot.RobotMap;
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

	// Arm characteristics
	private final double wristRatio = 360 / (140.0);
	private final double shoulderRatio = 360 / (512.0 / 3);

	// Arm PIDs
	private CANPIDController shoulderPID;
	private CANPIDController wristPID;

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

		// Setups arm PIDs
		shoulderPID = shoulderMotor.getPIDController();
		wristPID = wristMotor.getPIDController();

		shoulderPID.setP();
		shoulderPID.setI();
		shoulderPID.setD();

		wristPID.setP(0.00525);

		// Coverts motor rotations to degrees
		shoulderEncoder.setPositionConversionFactor(factor);
		wristEncoder.setPositionConversionFactor(factor);
	}

	/**
	 * @return the required feedforward for the shoulder to stay in place
	 */
	public double getShoulderFeedforward(double currentShoulderAngle) {
		double loadedShoulderAngle = 47.0;
		double angleDelta = Math.abs(loadedShoulderAngle - currentShoulderAngle);
		double shoulderFeedforward = ;

		return shoulderFeedforward * Math.cos(Math.toRadians(angleDelta));
	}

	/**
	 * Sets the arm to the desired overall position
	 * @param shoulderAngle the desired shoulder angle in degrees
	 * @param wristAngle the desired wrist angle in degrees
	 */
	public void goToPosition(double shoulderAngle, double wristAngle) {
		shoulderPID.setReference(shoulderAngle, ControlType.kPosition);
		wristPID.setReference(wristAngle, ControlType.kPosition);
	}

	@Override
	public void run() {
		shoulderPID.setFF(getShoulderFeedforward(shoulderEncoder.getPosition()));
	}

	@Override
	public void registerCommands() {
		new SubsystemCommand(this.registeredCommands, "jog_up") {
			@Override
			public void initialize() {}

			@Override
			public void execute() {
				goToPosition(shoulderEncoder.getPosition() + 3, wristEncoder.getPosition() + 3); //Shoulder previously +3.5
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
				goToPosition(shoulderEncoder.getPosition() - 3, wristEncoder.getPosition() - 3);
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
				return true;
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
				return true;
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
				return true;
			}

			@Override
			public void end() {}
		};



		new SubsystemCommand(this.registeredCommands, "station_position") {
			double shoulderAngle;
			double wristAngle;

			@Override
			public void initialize() {
				shoulderAngle = 7;
				wristAngle = 86;

				goToPosition(shoulderAngle, wristAngle);
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

		new SubsystemCommand(this.registeredCommands, "auton_lower_hatch") {
			double shoulderAngle;
			double wristAngle;

			@Override
			public void initialize() {
				shoulderAngle = 14;
				wristAngle = 90;

				goToPosition(shoulderAngle, wristAngle);
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

		new SubsystemCommand(this.registeredCommands, "go_to_position") {
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
				return true;
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
					shoulderAngle = 45;
					wristAngle = 160;
				} else {
					shoulderAngle = 4;
					wristAngle = 88;
				}

				goToPosition(shoulderAngle, wristAngle);
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

		new SubsystemCommand(this.registeredCommands, "cargo_station_score") {
			double shoulderAngle;
			double wristAngle;

			@Override
			public void initialize() {
				shoulderAngle = 82; //Previously 77
				wristAngle = 295;

				goToPosition(shoulderAngle, wristAngle);
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

		new SubsystemCommand(this.registeredCommands, "middle_score") {
			double shoulderAngle;
			double wristAngle;

			@Override
			public void initialize() {
				if(intake.getCargoState()) {
					shoulderAngle = 88;
					wristAngle = 200;
				} else {
					shoulderAngle = 66;
					wristAngle = 129;
				}

				goToPosition(shoulderAngle, wristAngle);
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

		new SubsystemCommand(this.registeredCommands, "upper_score") {
			double shoulderAngle;
			double wristAngle;

			@Override
			public void initialize() {
				if(intake.getCargoState()) {
					shoulderAngle = 122;
					wristAngle = 217;
				} else {
					shoulderAngle = 116;
					wristAngle = 185;
				}

				goToPosition(shoulderAngle, wristAngle);
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

		new SubsystemCommand(this.registeredCommands, "back_score") {
			double shoulderAngle;
			double wristAngle;

			@Override
			public void initialize() {
				if(intake.getHatchState()) {
					shoulderAngle = 130;
					wristAngle = 58;
					goToPosition(shoulderAngle, wristAngle);
				}
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
				return System.nanoTime() - timer >= 1000000000;
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