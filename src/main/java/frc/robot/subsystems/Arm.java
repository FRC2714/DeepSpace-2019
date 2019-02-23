package frc.robot.subsystems;

import java.util.ArrayList;
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
import frc.robot.util.PID;
import frc.robot.util.SubsystemCommand;
import frc.robot.util.SubsystemModule;

public class Arm extends SubsystemModule {

	private Intake intake;

	// Controls Processor
	private ControlsProcessor controlsProcessor;

	// Arm motors
	private CANSparkMax shoulderMotor = new CANSparkMax(7, MotorType.kBrushless);
	private CANSparkMax wristMotor = new CANSparkMax(8, MotorType.kBrushless);

	// PID controllers
	private CANPIDController shoulderMotorController = shoulderMotor.getPIDController();
	private CANPIDController wristPidController = wristMotor.getPIDController();

	// MAX encoders
	private CANEncoder shoulderMotorEncoder = shoulderMotor.getEncoder();
	private CANEncoder wristMotorEncoder = wristMotor.getEncoder();

	// Output encoders - Probably won't use these
	private Encoder shoulderOutputEncoder = new Encoder(RobotMap.p_shoulderEncoderA, RobotMap.p_shoulderEncoderB, true, EncodingType.k4X);
	// private Encoder wristOutputEncoder = new Encoder(RobotMap.p_wristEncoderA, RobotMap.p_wristEncoderB, true, EncodingType.k4X);

	
	// PID coefficients
	private final double sMinOutput = -1;
	private final double sMaxOutput = 1;
	private final double sP = 0.012; // 0.018
	private final double sI = 0.0; // 0.000038
	private final double sD = 0.0; // 0.001
	
	private final double wMinOutput = -1;
	private final double wMaxOutput = 1;
	private final double wP = 0.05;

	// Initialize PID
	private PID shoulderPID = new PID(sP, sI, sD);

	// All angles are in degrees
	private double currentShoulderAngle;
	private double currentWristAngle;
	private double currentShoulderSetpoint;
	private double currentWristSetpoint;
	
	// Shoulder linearization
	private final double shoulderHoldPower = 0.045;
	private final double parallelOffset = 47;
	private double shoulderFF;

	private final double maxDegreesPerSecond = 10;

	// Arm characteristics
	private final double wristRatio = -140;

	// Arm movement constants
	private final double armMaxVelocity = 220; // 120
	private final double armAcceleration = 2714; // 250
	private final double armJerk = 350; // 100

	// Array Lists
	private ArrayList<Double> shoulderPath;
	private ArrayList<Double> wristPath;

	private double endShoulderPosition;

	private boolean shoulderPathFinished;
	private boolean wristPathFinished;

	private ArrayList<Double> currentShoulderPosition;

	// Makes the arm give up
	private boolean giveUp;

	private boolean pathRunning;
	private boolean upperPosition;

	// Arm initialization
    public Arm(ControlsProcessor controlsProcessor) {
		intake = new Intake();
		controlsProcessor.registerController("Intake", intake);
		
		registerCommands();
		
		this.controlsProcessor = controlsProcessor;

		shoulderMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 2);

		wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
		wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);

		// Setup up PID coefficients
		wristPidController.setP(wP);
		wristPidController.setOutputRange(wMinOutput, wMaxOutput);

		wristMotorEncoder.setPosition(0);
		wristMotorEncoder.setPositionConversionFactor(360 / wristRatio);
	}
	
	/**
	 * Goes to the desired angle from the current angle on the shoulder
	 * @param desiredTheta Desired angle for the shoulder in degrees
	 */
	public void setShoulderAngle(double desiredTheta) {
		currentShoulderSetpoint = desiredTheta;
		// System.out.println("Shoulder Desired Theta: " + desiredTheta);
	}

	/**
	 * Goes to the desired angle from the current angle on the wrist
	 * @param desiredTheta Desired angle for the wrist in degrees
	 */
	public void setWristAngle(double desiredTheta) {
		double desiredMotorRotations = (desiredTheta / 360) * wristRatio;
		
		// System.out.println("Wrist Desired Theta: " + desiredTheta + " Desired Motor Rotations: " + desiredMotorRotations);
		wristPidController.setReference(desiredMotorRotations, ControlType.kPosition);
	}

	/**
	 * Calls generatePath with infinite max acceleration and infinite jerk for constant velocity
	 * @param startPosition in degrees
	 * @param endPosition in degrees
	 * @param velocity in degrees per second
	 * @return ArrayList of type double with every controlled point in the path
	 */
	public ArrayList<Double> generatePath(double startPosition, double endPosition, double velocity) {
		return generatePath(startPosition, endPosition, velocity, Double.MAX_VALUE, Double.MAX_VALUE);
	}

	/**
	 * Calls generatePath with infinite jerk for constant velocity
	 * @param startPosition in degrees
	 * @param endPosition in degrees
	 * @param maxVelocity in degrees per second
	 * @param acceleration in degrees per second per second
	 * @return ArrayList of type double with every controlled point in the path
	 */
	public ArrayList<Double> generatePath(double startPosition, double endPosition, double maxVelocity, double acceleration) {
		return generatePath(startPosition, endPosition, maxVelocity, acceleration, Double.MAX_VALUE);
	}

	/**
	 * Generates a path for the arm to follow while limiting jerk
	 * @param initialAngle in degrees
	 * @param desiredAngle in degrees
	 * @param maxVelocity in degrees per second
	 * @param maxAcceleration in degrees per second per second
	 * @param jerkConstant in degrees per second per second per second
	 * @return ArrayList of type double with every controlled point in the path
	 */
	public ArrayList<Double> generatePath(double initialAngle, double desiredAngle, double maxVelocity, double maxAcceleration, double jerkConstant) {
		int direction;
		
		if(desiredAngle - initialAngle < 0.2) { direction = -1; }
		else if(desiredAngle - initialAngle > 0.2) { direction = 1; }
		else { return new ArrayList<Double>(0); }

		double currentAcceleration = 0;
		double currentVelocity = 0;
		double angularDisplacement = 0;

		double period = controlsProcessor.getCommandPeriod();

		jerkConstant *= Math.pow(period, 3);
		maxAcceleration *= Math.pow(period, 2);
		maxVelocity *= period;

		ArrayList<Double> points = new ArrayList<Double>(0);
		points.add(initialAngle);
		points.add(desiredAngle);
		
		for(int i = 0; direction * (points.get(i + 1) - points.get(i)) >=  3 * direction * angularDisplacement; i++) {
			points.add(i + 1, points.get(i + 1) - angularDisplacement);
			points.add(i + 1, points.get(i) + angularDisplacement);

			currentAcceleration += jerkConstant;

			if(currentAcceleration > maxAcceleration)
				currentAcceleration = maxAcceleration;
			
			currentVelocity += currentAcceleration;

			if(currentVelocity > maxVelocity)
				currentVelocity = maxVelocity;
			
			angularDisplacement = direction * currentVelocity;
		}

		points.remove(0);
		points.remove(points.size() - 1);

		int midpoint = (points.size() / 2) - 1;
		points.add(midpoint + 1, (points.get(midpoint) + points.get(midpoint + 1)) / 2);

		// System.out.println("End Velocity: " + currentVelocity + "  Max Velocity: " + maxVelocity);
		// System.out.println("End Acceleration: " + currentAcceleration + "  Max Acceleration: " + maxAcceleration);
		// System.out.println("Total Time: " + (points.size() - 1) * period);

		return points;
	}

	public void pathVariableStart() {
		pathRunning = true;
		giveUp = false;

		shoulderPathFinished = false;
		wristPathFinished = false;

		upperPosition = false;

		intake.setAtPosition(false);
	}

	public void pathVariableEnd() {
		pathRunning = false;

		shoulderPath = new ArrayList<Double>(0);
		wristPath = new ArrayList<Double>(0);

		endShoulderPosition = shoulderMotorEncoder.getPosition();

		intake.setAtPosition(true);
	}

	/** 
	 * Locks the arm into a four bar configuration going up
	 */
	public void jogUp() {
		double currentDegreesPerPeriod = maxDegreesPerSecond * controlsProcessor.getCommandPeriod();
		currentShoulderSetpoint += currentDegreesPerPeriod;
		currentWristSetpoint += currentDegreesPerPeriod;

		setWristAngle(currentWristSetpoint);

		// System.out.println("Delta S: " + shoulderDelta + "\tW: " + wristDelta);
		// System.out.println("Up S:" + currentShoulderAngle + "\tW: " + currentWristAngle);
	}

	/** 
	 * Locks the arm into a four bar configuration going down
	 */
	public void jogDown() {
		double currentDegreesPerPeriod = maxDegreesPerSecond * controlsProcessor.getCommandPeriod();
		currentShoulderSetpoint -= currentDegreesPerPeriod;
		currentWristSetpoint -= currentDegreesPerPeriod;

		setWristAngle(currentWristSetpoint);

		// System.out.println("Delta S: " + shoulderDelta + "\tW: " + wristDelta);
		// System.out.println("Down S: " + currentShoulderAngle + "\tW: " + currentWristAngle);
	}

	public void shoulderFeedLogic() {
		if(pathRunning) {
			shoulderFF = shoulderHoldPower * Math.cos(Math.abs(currentShoulderSetpoint - parallelOffset) * (Math.PI / 180));
			shoulderPID.setP(sP);
			shoulderPID.setD(sD);
		} else {
			shoulderFF = 0;
			shoulderPID.setP(0.05);
			shoulderPID.setD(0.001);
		}
	}

	@Override
	public void run() {
		currentShoulderAngle = shoulderOutputEncoder.getDistance();
		currentWristAngle = wristMotorEncoder.getPosition();

		if(giveUp) { currentShoulderSetpoint = shoulderOutputEncoder.getDistance(); }
		
		shoulderFeedLogic();
		shoulderMotor.set(shoulderPID.getOutput(shoulderOutputEncoder.getDistance(), currentShoulderSetpoint) + shoulderFF);

		// System.out.println("Shoulder encoder: " + shoulderOutputEncoder.getDistance());
		// System.out.println("PID Output: " + shoulderPID.getOutput(currentShoulderAngle, currentShoulderSetpoint) + "\tEncoder Distance: " + shoulderOutputEncoder.getDistance());
	}

	@Override
	public void registerCommands() {

		new SubsystemCommand(this.registeredCommands, "get_arm_position") {
			
			@Override
			public void initialize() {
				System.out.println("WARNING: Is the PID off?");
				wristMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
				shoulderMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
			}

			@Override
			public void execute() { }

			@Override
			public boolean isFinished() {
				return false;
			}

			@Override
			public void end() {
				wristMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
				shoulderMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
				System.out.println("Shoulder Angle: " + currentShoulderAngle + "\tWrist Angle: " + currentWristAngle);
			}
		};

		new SubsystemCommand(this.registeredCommands, "start_position") {
			int iterator;

			@Override
			public void initialize() {
				pathVariableStart();

				if (!intake.getHatchState()) {
					shoulderPath = generatePath(currentShoulderAngle, 0,
							armMaxVelocity, armAcceleration, armJerk);

					wristPath = generatePath(currentWristAngle, 0,
							armMaxVelocity, armAcceleration, armJerk);

					iterator = 0;
				}
			}

			@Override
			public void execute() {
				iterator++;

				if (iterator < shoulderPath.size())
					setShoulderAngle(shoulderPath.get(iterator));
				else
					shoulderPathFinished = true;
				
				if (iterator < wristPath.size())
					setWristAngle(wristPath.get(iterator));
				else
					wristPathFinished = true;
			}

			@Override
			public boolean isFinished() {
				return shoulderPathFinished && wristPathFinished;
			}

			@Override
			public void end() {
				pathVariableEnd();
			}
		};


		new SubsystemCommand(this.registeredCommands, "floor_cargo_position") {
			int iterator;

			@Override
			public void initialize() {
				pathVariableStart();

				if (!intake.getHatchState() && !intake.getCargoState()) {
					shoulderPath = generatePath(currentShoulderAngle, 18,
							armMaxVelocity, armAcceleration, armJerk);

					wristPath = generatePath(currentWristAngle, 218,
							armMaxVelocity, armAcceleration, armJerk);

					iterator = 0;
				}
			}

			@Override
			public void execute() {
				iterator++;

				if (iterator < shoulderPath.size())
					setShoulderAngle(shoulderPath.get(iterator));
				else
					shoulderPathFinished = true;
				
				if (iterator < wristPath.size())
					setWristAngle(wristPath.get(iterator));
				else
					wristPathFinished = true;
			}

			@Override
			public boolean isFinished() {
				return shoulderPathFinished && wristPathFinished;
			}

			@Override
			public void end() {
				pathVariableEnd();
			}
		};


		new SubsystemCommand(this.registeredCommands, "floor_hatch_position") {
			int iterator;

			@Override
			public void initialize() {
				pathVariableStart();

				if (!intake.getHatchState() && !intake.getCargoState()) {
					shoulderPath = generatePath(currentShoulderAngle, 9,
							armMaxVelocity, armAcceleration, armJerk);

					wristPath = generatePath(currentWristAngle, 178,
							armMaxVelocity, armAcceleration, armJerk);

					iterator = 0;
				}
			}

			@Override
			public void execute() {
				iterator++;

				if (iterator < shoulderPath.size())
					setShoulderAngle(shoulderPath.get(iterator));
				else
					shoulderPathFinished = true;
				
				if (iterator < wristPath.size())
					setWristAngle(wristPath.get(iterator));
				else
					wristPathFinished = true;
			}

			@Override
			public boolean isFinished() {
				return shoulderPathFinished && wristPathFinished;
			}

			@Override
			public void end() {
				pathVariableEnd();
			}
		};


		new SubsystemCommand(this.registeredCommands, "station_position") {
			int iterator;

			@Override
			public void initialize() {
				pathVariableStart();

				shoulderPath = generatePath(currentShoulderAngle, 4,
						armMaxVelocity, armAcceleration, armJerk);

				wristPath = generatePath(currentWristAngle, 80,
						armMaxVelocity, armAcceleration, armJerk);
				// 86
				iterator = 0;
			}

			@Override
			public void execute() {
				iterator++;

				if (iterator < shoulderPath.size())
					setShoulderAngle(shoulderPath.get(iterator));
				else
					shoulderPathFinished = true;
				
				if (iterator < wristPath.size())
					setWristAngle(wristPath.get(iterator));
				else
					wristPathFinished = true;
			}

			@Override
			public boolean isFinished() {
				return shoulderPathFinished && wristPathFinished;
			}

			@Override
			public void end() {
				pathVariableEnd();
			}
		};


		new SubsystemCommand(this.registeredCommands, "lower_score") {
			int iterator;

			@Override
			public void initialize() {
				pathVariableStart();

				if (!intake.getHatchState() && intake.getCargoState()) {
					shoulderPath = generatePath(currentShoulderAngle, 46,
							armMaxVelocity, armAcceleration, armJerk);
					// 38
					wristPath = generatePath(currentWristAngle, 191,
							armMaxVelocity, armAcceleration, armJerk);

					iterator = 0;
				} else if (intake.getHatchState() && !intake.getCargoState()) {
					shoulderPath = generatePath(currentShoulderAngle, 4,
							armMaxVelocity, armAcceleration, armJerk);

					wristPath = generatePath(currentWristAngle, 72,
							armMaxVelocity, armAcceleration, armJerk);
					// 86
					iterator = 0;
				}
			}

			@Override
			public void execute() {
				iterator++;

				if (iterator < shoulderPath.size())
					setShoulderAngle(shoulderPath.get(iterator));
				else
					shoulderPathFinished = true;
				
				if (iterator < wristPath.size())
					setWristAngle(wristPath.get(iterator));
				else
					wristPathFinished = true;
			}

			@Override
			public boolean isFinished() {
				return shoulderPathFinished && wristPathFinished;
			}

			@Override
			public void end() {
				pathVariableEnd();
			}
		};


		new SubsystemCommand(this.registeredCommands, "middle_score") {
			int iterator;

			@Override
			public void initialize() {
				pathVariableStart();

				if (!intake.getHatchState() && intake.getCargoState()) {
					shoulderPath = generatePath(currentShoulderAngle, 79,
							armMaxVelocity, armAcceleration, armJerk);
					// 77
					wristPath = generatePath(currentWristAngle, 196,
							armMaxVelocity, armAcceleration, armJerk);
					// 227

					iterator = 0;
				} else if (intake.getHatchState() && !intake.getCargoState()) {
					shoulderPath = generatePath(currentShoulderAngle, 58,
							armMaxVelocity, armAcceleration, armJerk);
					// 50
					wristPath = generatePath(currentWristAngle, 118,
							armMaxVelocity, armAcceleration, armJerk);
					// 143
					iterator = 0;
				}
			}

			@Override
			public void execute() {
				iterator++;

				if (iterator < shoulderPath.size())
					setShoulderAngle(shoulderPath.get(iterator));
				else
					shoulderPathFinished = true;
				
				if (iterator < wristPath.size())
					setWristAngle(wristPath.get(iterator));
				else
					wristPathFinished = true;
			}

			@Override
			public boolean isFinished() {
				return shoulderPathFinished && wristPathFinished;
			}

			@Override
			public void end() {
				pathVariableEnd();
			}
		};


		new SubsystemCommand(this.registeredCommands, "upper_score") {
			int iterator;

			@Override
			public void initialize() {
				pathVariableStart();

				if (!intake.getHatchState() && intake.getCargoState()) {
					shoulderPath = generatePath(currentShoulderAngle, 120,
							armMaxVelocity, armAcceleration, armJerk);
					// 122
					wristPath = generatePath(currentWristAngle, 221,
							armMaxVelocity, armAcceleration, armJerk);
					// 233
					iterator = 0;
				} else if (intake.getHatchState() && !intake.getCargoState()) {
					shoulderPath = generatePath(currentShoulderAngle, 113,
							armMaxVelocity, armAcceleration, armJerk);
					// 91
					wristPath = generatePath(currentWristAngle, 200,
							armMaxVelocity, armAcceleration, armJerk);
					// 178
					iterator = 0;
				}
			}

			@Override
			public void execute() {
				iterator++;

				if (iterator < shoulderPath.size())
					setShoulderAngle(shoulderPath.get(iterator));
				else
					shoulderPathFinished = true;
				
				if (iterator < wristPath.size())
					setWristAngle(wristPath.get(iterator));
				else
					wristPathFinished = true;
			}

			@Override
			public boolean isFinished() {
				return shoulderPathFinished && wristPathFinished;
			}

			@Override
			public void end() {
				pathVariableEnd();
				upperPosition = true;
			}
		};


		new SubsystemCommand(this.registeredCommands, "back_score") {
			int iterator;

			@Override
			public void initialize() {
				pathVariableStart();

				if (intake.getHatchState() && !intake.getCargoState()) {
					shoulderPath = generatePath(currentShoulderAngle, 126,
							armMaxVelocity, armAcceleration, armJerk);

					wristPath = generatePath(currentWristAngle, 52,
							armMaxVelocity, armAcceleration, armJerk);

					iterator = 0;
				}
			}

			@Override
			public void execute() {
				iterator++;

				if (iterator < shoulderPath.size())
					setShoulderAngle(shoulderPath.get(iterator));
				else
					shoulderPathFinished = true;
				
				if (iterator < wristPath.size())
					setWristAngle(wristPath.get(iterator));
				else
					wristPathFinished = true;
			}

			@Override
			public boolean isFinished() {
				return shoulderPathFinished && wristPathFinished;
			}

			@Override
			public void end() {
				pathVariableEnd();
			}
		};


		new SubsystemCommand(this.registeredCommands, "go_to_position") {
			int iterator;

			@Override
			public void initialize() {
				pathVariableStart();

				shoulderPath = generatePath(currentShoulderAngle, Double.parseDouble(this.args[0]),
						armMaxVelocity, armAcceleration, armJerk);

				wristPath = generatePath(currentWristAngle, Double.parseDouble(this.args[1]),
						armMaxVelocity, armAcceleration, armJerk);

				iterator = 0;
			}

			@Override
			public void execute() {
				iterator++;

				if (iterator < shoulderPath.size())
					setShoulderAngle(shoulderPath.get(iterator));
				else
					shoulderPathFinished = true;
				
				if (iterator < wristPath.size())
					setWristAngle(wristPath.get(iterator));
				else
					wristPathFinished = true;
			}

			@Override
			public boolean isFinished() {
				return shoulderPathFinished && wristPathFinished;
			}

			@Override
			public void end() {
				pathVariableEnd();
			}
		};


		new SubsystemCommand(this.registeredCommands, "extake") {

			@Override
			public void initialize() {
				if(intake.getHatchState()) { intake.pumpMotor.set(0); }

				if(upperPosition && intake.checkCargoState()) {
					intake.cargoMotor.set(-0.5);
				}
				else if (!intake.checkPumpState()) {
					intake.cargoMotor.set(-1);
				}

				if(intake.getHatchState() || intake.getCargoState()) { giveUp = true; }
			}
			
			@Override
			public void execute() {
				intake.pumpRelease();
			}

			@Override
			public boolean isFinished() {
				return false;
			}

			@Override
			public void end() {
				intake.cargoMotor.set(0);
				intake.clearStates();
			}
		};


		new SubsystemCommand(this.registeredCommands, "jog_up") {

			@Override
			public void initialize() {
				pathVariableStart();
				currentShoulderSetpoint = currentShoulderAngle;
				currentWristSetpoint = currentWristAngle;
			}

			@Override
			public void execute() {
				jogUp();
			}

			@Override
			public boolean isFinished() {
				return false;
			}

			@Override
			public void end() {
				pathVariableEnd();
				System.out.println("Current Position: " + currentShoulderAngle);
			}
		};


		new SubsystemCommand(this.registeredCommands, "jog_down") {

			@Override
			public void initialize() {
				pathVariableStart();
				currentShoulderSetpoint = currentShoulderAngle;
				currentWristSetpoint = currentWristAngle;
			}

			@Override
			public void execute() {
				jogDown();
			}

			@Override
			public boolean isFinished() {
				return false;
			}

			@Override
			public void end() {
				pathVariableEnd();
			}
		};
    }

	@Override
	public void init() {
		intake.init();

		shoulderOutputEncoder.reset();
		// wristOutputEncoder.reset();

		shoulderOutputEncoder.setDistancePerPulse(45.0 / 512);
		// wristOutputEncoder.setDistancePerPulse(wristScaler);

		shoulderPID.setOutputLimits(sMinOutput, sMaxOutput);

		currentShoulderAngle = 0;
		currentWristAngle = 0;
		currentShoulderSetpoint = 0;

		shoulderFF = 0;

		shoulderPath = new ArrayList<Double>(0);
		wristPath = new ArrayList<Double>(0);

		shoulderMotorEncoder.setPosition(0);
		wristMotorEncoder.setPosition(0);

		shoulderPathFinished = false;
		wristPathFinished = false;

		giveUp = false;
		upperPosition = false;

		shoulderMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		wristMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
	}

	@Override
	public void destruct() {
		System.out.println("ARM DESTRUCT");

		shoulderMotor.set(0);
		wristMotor.set(0);

		System.out.println(currentShoulderPosition);

		shoulderMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		wristMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		
		intake.destruct();
	}
}