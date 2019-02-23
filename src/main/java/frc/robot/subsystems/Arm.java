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

	// Arm motors
	private final CANSparkMax shoulderMotor = new CANSparkMax(7, MotorType.kBrushless);
	private final CANSparkMax wristMotor = new CANSparkMax(8, MotorType.kBrushless);
	
	// Initialize arm encoders
	private Encoder shoulderEncoder = new Encoder(RobotMap.p_shoulderEncoderA, RobotMap.p_shoulderEncoderB, true, EncodingType.k4X);
	private CANEncoder wristEncoder = wristMotor.getEncoder();

	// Shoulder linearization
	private final double shoulderLoadPosition = 47.0; // In degrees
	private final double shoulderFeedforward = 0.045; // In degrees per second

	// PID coefficients for the shoulder
	private final double sMinOutput = -1;
	private final double sMaxOutput = 1;
	private final double sRate = 0.025;
	private final double sP = 0.00525;
	private final double sI = 0.000005;
	private final double sD = 0.0005;
	
	// PID coefficients for the wrist
	private final double wMinOutput = -1;
	private final double wMaxOutput = 1;
	private final double wP = 0.00525;
	private final double wI = 0.0;
	private final double wD = 0.0;

	// PID controllers
	private PID shoulderPID = new PID(sP, sI, sD);
	private CANPIDController wristPID =  wristMotor.getPIDController();

	// Arm characteristics
	private final double wristRatio = 140;

	// Current arm angles in degrees
	private double currentShoulderAngle;
	private double currentWristAngle;

	// Desired arm angles in degrees
	private double desiredShoulderAngle;
	private double desiredWristAngle;

	// Test variables
	private ArrayList<Double> shoulderMovement;
	private ArrayList<Double> wristMovement;

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
		wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
		wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
		
		// Inverts wrist motor direction
		wristMotor.setInverted(true);

		// Set encoder conversion factors
		shoulderEncoder.setDistancePerPulse(45.0 / 512);
		wristEncoder.setPositionConversionFactor(360.0 / wristRatio);
		
		// Set PID coefficients
		shoulderPID.setOutputLimits(sMinOutput, sMaxOutput);
		shoulderPID.setOutputRampRate(sRate);
		shoulderPID.setP(sP);
		shoulderPID.setI(sI);
		shoulderPID.setD(sD);
		wristPID.setOutputRange(wMinOutput, wMaxOutput);
		wristPID.setP(wP);
		wristPID.setI(wI);
		wristPID.setD(wD);

		// Initialize ArrayLists
		shoulderMovement = new ArrayList<Double>(0);
		wristMovement = new ArrayList<Double>(0);
	}

	/**
	 * Sets currentShoulderAngle and currentWristAngle to the encoder values
	 */
	public void updateCurrentAngles() {
		currentShoulderAngle = shoulderEncoder.getDistance();
		currentWristAngle = wristEncoder.getPosition();
	}

	/**
	 * @return the required feedforward for the shoulder to stay in place
	 */
	public double getShoulderFeedforward() {
		double angleDelta = Math.abs(shoulderLoadPosition - currentShoulderAngle);
		angleDelta = Math.toRadians(angleDelta);

		return (Math.cos(angleDelta) * shoulderFeedforward);
	}

	/**
	 * Uses the shoulder PID and feedforward value to set shoulder motor power
	 */
	public void setShoulderPower() {
		double motorPower = shoulderPID.getOutput(currentShoulderAngle, desiredShoulderAngle);
		motorPower += getShoulderFeedforward();

		shoulderMotor.set(motorPower);
	}

	/**
	 * Plugs in desired wrist angle to the wrist PID
	 */
	public void setWristPower() {
		wristPID.setReference(desiredWristAngle, ControlType.kPosition);
	}

	/**
	 * Sets the arm to the desired overall position
	 * @param shoulderAngle the desired shoulder angle in degrees
	 * @param wristAngle the desired wrist angle in degrees
	 */
	public void goToPosition(double shoulderAngle, double wristAngle) {
		desiredShoulderAngle = shoulderAngle;
		desiredWristAngle = wristAngle;
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

		return atShoulderAngle && atWristAngle;
	}

	/**
	 * 
	 */
	public void trackMovement() {
		if(desiredShoulderAngle != 0 || desiredWristAngle != 0) {
			shoulderMovement.add(currentShoulderAngle);
			wristMovement.add(currentWristAngle);
		}
	}

	/**
	 * 
	 */
	public void printMovement() {
		System.out.println("Shoulder\t" + shoulderMovement);
		System.out.println("Wrist\t" + wristMovement);
	}

	@Override
	public void run() {
		updateCurrentAngles();

		setShoulderPower();
		setWristPower();

		trackMovement();
	}

	@Override
	public void registerCommands() {
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
			public void end() {
				System.out.println("atPosition = true");
			}
		};
    }

	@Override
	public void init() {
		intake.init();

		shoulderMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		wristMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		
		wristEncoder.setPosition(0);

		shoulderPID.reset();

		desiredShoulderAngle = 0;
		desiredWristAngle = 0;

		shoulderMovement = new ArrayList<Double>(0);
		wristMovement = new ArrayList<Double>(0);
	}

	@Override
	public void destruct() {
		intake.destruct();

		shoulderMotor.set(0);
		wristMotor.set(0);
		
		shoulderMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		wristMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

		printMovement();
	}
}