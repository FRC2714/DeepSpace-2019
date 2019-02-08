package frc.robot.subsystems;

import java.util.ArrayList;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import frc.robot.RobotMap;
import frc.robot.util.ControlsProcessor;
import frc.robot.util.SubsystemCommand;
import frc.robot.util.SubsystemModule;

public class Arm extends SubsystemModule {

	// Controls Processor
	private ControlsProcessor controlsProcessor;

	// Arm motors
	private CANSparkMax shoulderMotor = new CANSparkMax(7, MotorType.kBrushless);
	private CANSparkMax wristMotor = new CANSparkMax(8, MotorType.kBrushless);
	
	// PID controllers
	private CANPIDController shoulderPidController = shoulderMotor.getPIDController();
	private CANPIDController wristPidController = wristMotor.getPIDController();

	// MAX encoders
	private CANEncoder shoulderMotorEncoder = shoulderMotor.getEncoder();
	private CANEncoder wristMotorEncoder = wristMotor.getEncoder();

	// Output encoders
	private Encoder shoulderOutputEncoder = new Encoder(RobotMap.p_shoulderEncoderA, RobotMap.p_shoulderEncoderB, true, EncodingType.k4X);
	private Encoder wristOutputEncoder = new Encoder(RobotMap.p_wristEncoderA, RobotMap.p_wristEncoderB, true, EncodingType.k4X);

	// PID coefficients
	private final double sMinOutput = -1;
	private final double sMaxOutput = 1;
	private final double sP = 0.65;
	private final double sI = 0;
	private final double sD = 0;
	private final double sIS = 0;
	private final double sFF = 0;

	private final double wMinOutput = -1;
	private final double wMaxOutput = 1;
	private final double wP = 0.18;
	private final double wI = 0;
	private final double wD = 0;
	private final double wIS = 0;
	private final double wFF = 0;

	// All angles are in degrees
	private double shoulderOffset = 0;
	private double wristOffset = 0;
	private double currentShoulderAngle = 0;
	private double currentWristAngle = 0;
	private final double maxDegreesPerSecond = 30;

	// Arm characteristics
	private final double shoulderRatio = 512.0/3;
	private final double wristRatio = -140;

	// Array Lists
	private ArrayList<Double> shoulderPath;
	private ArrayList<Double> wristPath;

	private boolean shoulderPathFinished = false;
	private boolean wristPathFinished = false;

	// Arm states
	private boolean startPosition = false;
	private boolean floorIntake = false;
	private boolean stationIntake = false;
	private boolean cargoPosition = false;
	private boolean hatchPosition = false;
	
	//TODO: Make real sensors instead of these
	private boolean cargoSensor = false;
	private boolean hatchSensor = false;

	// Arm initialization
    public Arm(ControlsProcessor controlsProcessor) {
		registerCommands();
		
		this.controlsProcessor = controlsProcessor;

		// Setup up PID coefficients
		shoulderPidController.setP(sP);
		shoulderPidController.setI(sI);
		shoulderPidController.setD(sD);
		shoulderPidController.setIZone(sIS);
		shoulderPidController.setFF(sFF);
		shoulderPidController.setOutputRange(sMinOutput, sMaxOutput);

		wristPidController.setP(wP);
		wristPidController.setI(wI);
		wristPidController.setD(wD);
		wristPidController.setIZone(wIS);
		wristPidController.setFF(wFF);
		wristPidController.setOutputRange(wMinOutput, wMaxOutput);
	}
	
	/**
	 * Goes to the desired angle from the current angle on the shoulder
	 * @param desiredTheta Desired angle for the shoulder in degrees
	 */
	public void setShoulderAngle(double desiredTheta) {
		double desiredMotorRotations = ((shoulderOffset + desiredTheta) / 360) * shoulderRatio;

		// System.out.println("Shoulder Desired Theta: " + desiredTheta + " Desired Motor Rotations: " + desiredMotorRotations);
		shoulderPidController.setReference(desiredMotorRotations, ControlType.kPosition);
	}

	/**
	 * Goes to the desired angle from the current angle on the wrist
	 * @param desiredTheta Desired angle for the wrist in degrees
	 */
	public void setWristAngle(double desiredTheta) {
		double desiredMotorRotations = ((wristOffset + desiredTheta) / 360) * wristRatio;
		
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
		
		if(desiredAngle - initialAngle < 0) { direction = -1; }
		else if(desiredAngle - initialAngle > 0) { direction = 1; }
		else { return new ArrayList<Double>(); }

		double currentAcceleration = 0;
		double currentVelocity = 0;
		double angularDisplacement = 0;

		double period = controlsProcessor.getCommandPeriod();

		jerkConstant *= Math.pow(period, 3);
		maxAcceleration *= Math.pow(period, 2);
		maxVelocity *= period;

		ArrayList<Double> points = new ArrayList<Double>();
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

		System.out.println("End Velocity: " + currentVelocity + "  Max Velocity: " + maxVelocity);
		System.out.println("End Acceleration: " + currentAcceleration + "  Max Acceleration: " + maxAcceleration);
		System.out.println("Total Time: " + (points.size() - 1) * period);

		return points;
	}

	/** 
	 * Locks the arm into a four bar configuration going up
	 */
	public void jogUp() {

		double currentDegreesPerPeriod = maxDegreesPerSecond * controlsProcessor.getCommandPeriod();
		double shoulderDelta = currentShoulderAngle + (currentDegreesPerPeriod);
		double wristDelta = currentWristAngle + (currentDegreesPerPeriod);

		setShoulderAngle(shoulderDelta);
		setWristAngle(wristDelta);

		// System.out.println("Delta S: " + shoulderDelta + "\tW: " + wristDelta);
		// System.out.println("Up S:" + currentShoulderAngle + "\tW: " + currentWristAngle);
	}

	/** 
	 * Locks the arm into a four bar configuration going down
	 */
	public void jogDown() {

		double currentDegreesPerPeriod = maxDegreesPerSecond * controlsProcessor.getCommandPeriod();
		double shoulderDelta = currentShoulderAngle - currentDegreesPerPeriod;
		double wristDelta = currentWristAngle - currentDegreesPerPeriod;

		setShoulderAngle(shoulderDelta);
		setWristAngle(wristDelta);

		// System.out.println("Delta S: " + shoulderDelta + "\tW: " + wristDelta);
		// System.out.println("Down S: " + currentShoulderAngle + "\tW: " + currentWristAngle);
	}

	@Override
	public void run() {
		currentShoulderAngle = ((shoulderMotorEncoder.getPosition() / shoulderRatio) * 360) - shoulderOffset;
		currentWristAngle = ((wristMotorEncoder.getPosition() / wristRatio) * 360) - wristOffset;

		//TODO: set sensor booleans to sensor readout
	}

	@Override
	public void registerCommands() {

		new SubsystemCommand(this.registeredCommands, "start_position") {

			int iterator;

			@Override
			public void initialize() {
				if (!hatchSensor) {
					startPosition = true;
					floorIntake = false;
					stationIntake = false;
					cargoPosition = false;
					hatchPosition = false;

					shoulderPathFinished = false;
					wristPathFinished = false;

					double shoulderMaxVelocity = 70;
					double wristMaxVelocity = 70;

					shoulderPath = generatePath(currentShoulderAngle, 0,
							shoulderMaxVelocity, shoulderMaxVelocity * 4, shoulderMaxVelocity * 16);

					wristPath = generatePath(currentWristAngle, 0,
							wristMaxVelocity, wristMaxVelocity * 4, wristMaxVelocity * 16);

					iterator = 0;
				} else {
					end(); //TODO: test that this prevents execute
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
				shoulderPath = new ArrayList<Double>();
				wristPath = new ArrayList<Double>();
			}
		};

		new SubsystemCommand(this.registeredCommands, "floor_intake") {

			int iterator;

			@Override
			public void initialize() {
				if (!hatchSensor && !cargoSensor) {
					startPosition = false;
					floorIntake = true;
					stationIntake = false;
					cargoPosition = false;
					hatchPosition = false;

					shoulderPathFinished = false;
					wristPathFinished = false;

					double shoulderMaxVelocity = 70;
					double wristMaxVelocity = 70;

					shoulderPath = generatePath(currentShoulderAngle, 22.75,
							shoulderMaxVelocity, shoulderMaxVelocity * 4, shoulderMaxVelocity * 16);

					wristPath = generatePath(currentWristAngle, 182,
							wristMaxVelocity, wristMaxVelocity * 4, wristMaxVelocity * 16);

					iterator = 0;
				} else {
					end(); //TODO: test that this prevents execute
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
				shoulderPath = new ArrayList<Double>();
				wristPath = new ArrayList<Double>();
			}
		};

		new SubsystemCommand(this.registeredCommands, "station_intake") {

			int iterator;

			@Override
			public void initialize() {
				if (!hatchSensor && !cargoSensor) {
					startPosition = false;
					floorIntake = false;
					stationIntake = true;
					cargoPosition = false;
					hatchPosition = false;

					shoulderPathFinished = false;
					wristPathFinished = false;

					double shoulderMaxVelocity = 70;
					double wristMaxVelocity = 70;
					
					shoulderPath = generatePath(currentShoulderAngle, 15,
							shoulderMaxVelocity, shoulderMaxVelocity * 4, shoulderMaxVelocity * 16);

					wristPath = generatePath(currentWristAngle, 80,
							wristMaxVelocity, wristMaxVelocity * 4, wristMaxVelocity * 16);

					iterator = 0;
				} else {
					end(); //TODO: test that this prevents execute
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
				shoulderPath = new ArrayList<Double>();
				wristPath = new ArrayList<Double>();
			}
		};

		new SubsystemCommand(this.registeredCommands, "lower_score") {

			int iterator;

			@Override
			public void initialize() {
				if ((floorIntake || cargoPosition) && !hatchSensor && cargoSensor) {
					startPosition = false;
					floorIntake = false;
					stationIntake = false;
					cargoPosition = true;
					hatchPosition = false;

					shoulderPathFinished = false;
					wristPathFinished = false;

					double shoulderMaxVelocity = 70;
					double wristMaxVelocity = 70;
					
					shoulderPath = generatePath(currentShoulderAngle, 53,
							shoulderMaxVelocity, shoulderMaxVelocity * 4, shoulderMaxVelocity * 16);

					wristPath = generatePath(currentWristAngle, 195,
							wristMaxVelocity, wristMaxVelocity * 4, wristMaxVelocity * 16);

					iterator = 0;
				} else {
					end(); //TODO: test that this prevents execute
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
				shoulderPath = new ArrayList<Double>();
				wristPath = new ArrayList<Double>();
			}
		};

		new SubsystemCommand(this.registeredCommands, "cargo_middle_position") {

			int iterator;

			@Override
			public void initialize() {
				if ((floorIntake || cargoPosition) && !hatchSensor && cargoSensor) {
					startPosition = false;
					floorIntake = false;
					stationIntake = false;
					cargoPosition = true;
					hatchPosition = false;

					shoulderPathFinished = false;
					wristPathFinished = false;

					double shoulderMaxVelocity = 70;
					double wristMaxVelocity = 70;
					
					shoulderPath = generatePath(currentShoulderAngle, 85,
							shoulderMaxVelocity, shoulderMaxVelocity * 4, shoulderMaxVelocity * 16);

					wristPath = generatePath(currentWristAngle, 200,
							wristMaxVelocity, wristMaxVelocity * 4, wristMaxVelocity * 16);

					iterator = 0;
				} else {
					end(); //TODO: test that this prevents execute
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
				shoulderPath = new ArrayList<Double>();
				wristPath = new ArrayList<Double>();
			}
		};

		new SubsystemCommand(this.registeredCommands, "cargo_upper_position") {

			int iterator;

			@Override
			public void initialize() {
				if ((floorIntake || cargoPosition) && !hatchSensor && cargoSensor) {
					startPosition = false;
					floorIntake = false;
					stationIntake = false;
					cargoPosition = true;
					hatchPosition = false;

					shoulderPathFinished = false;
					wristPathFinished = false;

					double shoulderMaxVelocity = 70;
					double wristMaxVelocity = 70;
					
					shoulderPath = generatePath(currentShoulderAngle, 110,
							shoulderMaxVelocity, shoulderMaxVelocity * 4, shoulderMaxVelocity * 16);

					wristPath = generatePath(currentWristAngle, 230.8,
							wristMaxVelocity, wristMaxVelocity * 4, wristMaxVelocity * 16);

					iterator = 0;
				} else {
					end(); //TODO: test that this prevents execute
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
				shoulderPath = new ArrayList<Double>();
				wristPath = new ArrayList<Double>();
			}
		};

		new SubsystemCommand(this.registeredCommands, "hatch_lower_position") {

			int iterator;

			@Override
			public void initialize() {
				if ((floorIntake || cargoPosition) && !hatchSensor && cargoSensor) {
					startPosition = false;
					floorIntake = false;
					stationIntake = false;
					cargoPosition = true;
					hatchPosition = false;

					shoulderPathFinished = false;
					wristPathFinished = false;

					double shoulderMaxVelocity = 70;
					double wristMaxVelocity = 70;
					
					shoulderPath = generatePath(currentShoulderAngle, 110,
							shoulderMaxVelocity, shoulderMaxVelocity * 4, shoulderMaxVelocity * 16);

					wristPath = generatePath(currentWristAngle, 230.8,
							wristMaxVelocity, wristMaxVelocity * 4, wristMaxVelocity * 16);

					iterator = 0;
				} else {
					end(); //TODO: test that this prevents execute
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
				shoulderPath = new ArrayList<Double>();
				wristPath = new ArrayList<Double>();
			}
		};

		new SubsystemCommand(this.registeredCommands, "go_to_position") {

			int iterator;

			@Override
			public void initialize() {
				shoulderPathFinished = false;
				wristPathFinished = false;

				double shoulderMaxVelocity = Double.parseDouble(this.args[1]);
				double wristMaxVelocity = Double.parseDouble(this.args[3]);

				shoulderPath = generatePath(currentShoulderAngle, Double.parseDouble(this.args[0]),
						shoulderMaxVelocity, shoulderMaxVelocity * 4, shoulderMaxVelocity * 16);

				wristPath = generatePath(currentWristAngle, Double.parseDouble(this.args[2]),
						wristMaxVelocity, wristMaxVelocity * 4, wristMaxVelocity * 16);

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
				shoulderPath = new ArrayList<Double>();
				wristPath = new ArrayList<Double>();
			}
		};

		new SubsystemCommand(this.registeredCommands, "jog_up") {

			@Override
			public void initialize() {

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

			}
		};

		new SubsystemCommand(this.registeredCommands, "jog_down") {

			@Override
			public void initialize() {
			
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

			}
		};
    }

	@Override
	public void init() {
		// shoulderOutputEncoder.reset();
		// wristOutputEncoder.reset();

		// shoulderOutputEncoder.setDistancePerPulse(shoulderScaler);
		// wristOutputEncoder.setDistancePerPulse(wristScaler);

		currentShoulderAngle = 0;
		currentWristAngle = 0;

		shoulderOffset = (shoulderMotorEncoder.getPosition() / shoulderRatio) * 360;
		wristOffset = (wristMotorEncoder.getPosition() / wristRatio) * 360;

		shoulderMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		wristMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
	}

	@Override
	public void destruct() {
		shoulderMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		wristMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		
		shoulderMotor.set(0);
		wristMotor.set(0);
	}
}