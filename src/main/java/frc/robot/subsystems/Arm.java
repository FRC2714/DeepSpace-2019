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
	private final double sP = 0;
	private final double sI = 0;
	private final double sD = 0;
	private final double sIS = 0;
	private final double sFF = 0;

	private final double wMinOutput = -1;
	private final double wMaxOutput = 1;
	private final double wP = 0;
	private final double wI = 0;
	private final double wD = 0;
	private final double wIS = 0;
	private final double wFF = 0;

	private double shoulderOffset = 0.0;
	private double wristOffset = 0.0;

	// Arm characteristics
	private final double shoulderRatio = 512.0/3;
	private final double wristRatio = 140;

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
	 * TODO: Offset might be wrong
	 * @param desiredTheta Desired angle for the shoulder in degrees
	 */
	public void setShoulderAngle(double desiredTheta) {
		desiredTheta = (shoulderOffset + desiredTheta) / shoulderRatio * 360;

		shoulderPidController.setReference(desiredTheta, ControlType.kPosition);
	}

	/**
	 * Goes to the desired angle from the current angle on the wrist
	 * TODO: Offset might be wrong
	 * @param desiredTheta Desired angle for the wrist in degrees
	 */
	public void setWristAngle(double desiredTheta) {
		desiredTheta = (wristOffset + desiredTheta) / wristRatio * 360;
		
		wristPidController.setReference(desiredTheta, ControlType.kPosition);
	}

	/**
	 * Generates a path for the arm to follow
	 * @param startPosition in degrees
	 * @param endPosition in degrees
	 * @param revolutionsPerSecond Velocity in RPS
	 * @return ArrayList of type double with every point in the path
	 */
	public ArrayList<Double> generatePath(double startPosition, double endPosition, double revolutionsPerSecond) {
		int direction = (int)(endPosition - startPosition);
		direction /= Math.abs(direction);

		double degreesPerSecond = revolutionsPerSecond * 360;
		double positionDelta = direction * degreesPerSecond * controlsProcessor.getActualPeriod();
		
		ArrayList<Double> points = new ArrayList<Double>(1);
		points.add(0, endPosition);

		if(startPosition < endPosition) {
			while(points.get(0) >= startPosition) {
				points.add(0, points.get(0) - positionDelta);
			}
		}
		else {
			while(points.get(0) <= startPosition) {
				points.add(0, points.get(0) - positionDelta);
			}
		}
		
		points.set(0, startPosition);

		return points;
	}

	@Override
	public void run() {
		
    }

	@Override
	public void registerCommands() {

        new SubsystemCommand(this.registeredCommands, "go_to_position_basic") {

			int iterator;

			@Override
			public void initialize() {
				shoulderPath = generatePath(Double.parseDouble(this.args[0]),
						Double.parseDouble(this.args[1]), Double.parseDouble(this.args[2]));

				wristPath = generatePath(Double.parseDouble(this.args[3]),
						Double.parseDouble(this.args[4]), Double.parseDouble(this.args[5]));

				iterator = 0;
			}

			@Override
			public void execute() {
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
				destruct();
			}
		};
    }

	@Override
	public void init() {
		// shoulderOutputEncoder.reset();
		// wristOutputEncoder.reset();

		// shoulderOutputEncoder.setDistancePerPulse(shoulderScaler);
		// wristOutputEncoder.setDistancePerPulse(wristScaler);

		shoulderMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		wristMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
	}

	@Override
	public void destruct() {
		shoulderMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		wristMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		
		shoulderMotor.set(0);
		wristMotor.set(0);
	}
}