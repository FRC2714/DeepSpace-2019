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
	private final double sP = 0.1;
	private final double sI = 0;
	private final double sD = 0;
	private final double sIS = 0;
	private final double sFF = 0;

	private final double wMinOutput = -1;
	private final double wMaxOutput = 1;
	private final double wP = 0.1;
	private final double wI = 0;
	private final double wD = 0;
	private final double wIS = 0;
	private final double wFF = 0;

	// All angles are in degrees
	private double shoulderOffset = 0;
	private double wristOffset = 0;
	private double currentShoulderAngle = 0;
	private double currentWristAngle = 0;
	private double currentDegreesPerSecond = 0;
	private final double maxDegreesPerSecond = 5;

	// Arm characteristics
	private final double shoulderRatio = 512.0/3;
	private final double wristRatio = -140;

	// Array Lists
	private ArrayList<Double> shoulderPath;
	private ArrayList<Double> wristPath;

	private boolean shoulderPathFinished = false;
	private boolean wristPathFinished = false;

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
	 * Generates a path for the arm to follow
	 * @param startPosition in degrees
	 * @param endPosition in degrees
	 * @param degreesPerSecond Velocity in degrees per second
	 * @return ArrayList of type double with every point in the path
	 */
	public ArrayList<Double> generatePath(double startPosition, double endPosition, double degreesPerSecond) {
		int direction = (int)(endPosition - startPosition);
		
		// Sets the direction of the path to positive or negative (returns 1 or -1 for math)
		if (Math.abs(direction) != 0)
			direction /= Math.abs(direction);
		else // Prevents nullPointerException
			return new ArrayList<Double>();

		// Calculates the change in position in degrees for each period
		double positionDelta = direction * degreesPerSecond * controlsProcessor.getCommandPeriod();
		
		ArrayList<Double> points = new ArrayList<Double>();
		points.add(0, endPosition);

		// Generates the points between the start and end positions (starting from end and going back)
		if(startPosition < endPosition) {
			while(points.get(0) >= startPosition) {
				points.add(0, points.get(0) - positionDelta);
			}
		} else {
			while(points.get(0) <= startPosition) {
				points.add(0, points.get(0) - positionDelta);
			}
		}
		
		points.set(0, startPosition);

		return points;
	}

	/**
	 * Generates a path for the arm to follow while limiting acceleration
	 * @param startPosition in degrees
	 * @param endPosition in degrees
	 * @param maxDegreesPerSecond Max velocity in degrees per second
	 * @param acceleration Acceleration in degrees per second per second
	 * @return ArrayList of type double with every point in the path
	 */
	public ArrayList<Double> generatePath(double startPosition, double endPosition, double maxDegreesPerSecond, double acceleration) {
		int direction = (int)(endPosition - startPosition);
		
		// Sets the direction of the path to positive or negative (returns 1 or -1 for math)
		if (Math.abs(direction) != 0)
			direction /= Math.abs(direction);
		else // Prevents nullPointerException
			return new ArrayList<Double>();

		acceleration *= controlsProcessor.getCommandPeriod();

		double velocity = currentDegreesPerSecond;
		double positionDelta = direction * velocity * controlsProcessor.getCommandPeriod();

		ArrayList<Double> points = new ArrayList<Double>();
		points.add(startPosition);
		points.add(endPosition);

		// Keeps adding points until they meet in the middle
		for (int i = 0; points.get(i) * direction < points.get(i + 1) * direction; i++) {
			velocity += acceleration;

			if (velocity >= maxDegreesPerSecond)
				positionDelta = direction * maxDegreesPerSecond * controlsProcessor.getCommandPeriod();
			else
				positionDelta = direction * velocity * controlsProcessor.getCommandPeriod();

			points.add(i + 1, points.get(i + 1) - positionDelta);
			points.add(i + 1, points.get(i) + positionDelta);
		}

		return points;
	}

	/** 
	 * Locks the arm into a four bar configuration going up
	 */
	public void jogUp() {
		currentDegreesPerSecond = maxDegreesPerSecond;

		double currentDegreesPerPeriod = currentDegreesPerSecond * controlsProcessor.getCommandPeriod();

		currentShoulderAngle += currentDegreesPerPeriod;
		currentWristAngle -= currentDegreesPerPeriod;

		setShoulderAngle(currentShoulderAngle);
		setWristAngle(currentWristAngle);
	}

	/** 
	 * Locks the arm into a four bar configuration going down
	 */
	public void jogDown() {
		currentDegreesPerSecond = maxDegreesPerSecond;

		double currentDegreesPerPeriod = currentDegreesPerSecond * controlsProcessor.getCommandPeriod();

		currentShoulderAngle -= currentDegreesPerPeriod;
		currentWristAngle += currentDegreesPerPeriod;

		setShoulderAngle(currentShoulderAngle);
		setWristAngle(currentWristAngle);
	}

	@Override
	public void run() {
		currentShoulderAngle = ((shoulderMotorEncoder.getPosition() / shoulderRatio) * 360) - shoulderOffset;
		currentWristAngle = ((wristMotorEncoder.getPosition() / wristRatio) * 360) - wristOffset;
	}

	@Override
	public void registerCommands() {

        new SubsystemCommand(this.registeredCommands, "go_to_position_basic") {

			int iterator;

			@Override
			public void initialize() {
				shoulderPathFinished = false;
				wristPathFinished = false;

				shoulderPath = generatePath(currentShoulderAngle,
						Double.parseDouble(this.args[0]), Double.parseDouble(this.args[1]));

				wristPath = generatePath(currentWristAngle,
						Double.parseDouble(this.args[2]), Double.parseDouble(this.args[3]));

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
				
			}
		};

		new SubsystemCommand(this.registeredCommands, "go_to_position") {

			int iterator;

			@Override
			public void initialize() {
				shoulderPathFinished = false;
				wristPathFinished = false;

				shoulderPath = generatePath(currentShoulderAngle, Double.parseDouble(this.args[0]),
						Double.parseDouble(this.args[1]), Double.parseDouble(this.args[2]));

				wristPath = generatePath(currentWristAngle, Double.parseDouble(this.args[3]),
						Double.parseDouble(this.args[4]), Double.parseDouble(this.args[5]));

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

		new SubsystemCommand(this.registeredCommands, "print_arm") {

			@Override
			public void initialize() {
				shoulderMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
				wristMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

				System.out.println("Current Shoulder Angle: " + currentShoulderAngle + " Current Wrist Angle: " + currentWristAngle);
			}

			@Override
			public void execute() {
				// System.out.println("Current Shoulder Angle: " + currentShoulderAngle + " Current Wrist Angle: " + currentWristAngle);
			}

			@Override
			public boolean isFinished() {
				return false;
			}

			@Override
			public void end() {
				System.out.println("Current Shoulder Angle: " + currentShoulderAngle + " Current Wrist Angle: " + currentWristAngle);
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
		System.out.println("ARM DESTRUCT CALLED");

		

		shoulderMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		wristMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		
		shoulderMotor.set(0);
		wristMotor.set(0);
	}
}