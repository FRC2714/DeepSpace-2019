package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import frc.robot.RobotMap;
import frc.robot.util.SubsystemCommand;
import frc.robot.util.SubsystemModule;

public class Arm extends SubsystemModule {

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

	// Arm characteristics
	private final double shoulderRatio = 512.0/3;
	private final double wristRatio = 140;

	// Arm initialization
    public Arm() {
		registerCommands();
		
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

    @Override public void run() {
		double initialShoulderTheta = 0.0;
		double initialWristTheta = 0.0;


    }

    @Override public void registerCommands() {

        new SubsystemCommand(this.registeredCommands, "example_command") {

			@Override
			public void initialize() {

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

			}
		};
    }

	@Override
	public void init() {
		shoulderOutputEncoder.reset();
		wristOutputEncoder.reset();

		shoulderOutputEncoder.setDistancePerPulse(distancePerPulse);
		wristOutputEncoder.setDistancePerPulse(distancePerPulse);

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