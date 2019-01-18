package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.util.DrivingController;
import frc.robot.util.Odometer;
import frc.robot.util.SubsystemCommand;
import frc.robot.util.SubsystemModule;

public class DriveTrain extends SubsystemModule {

	//DriveTrain Motors
    private CANSparkMax lMotor0 = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax lMotor1 = new CANSparkMax(1, MotorType.kBrushless);
    private CANSparkMax lMotor2 = new CANSparkMax(2, MotorType.kBrushless);
    private CANSparkMax rMotor0 = new CANSparkMax(3, MotorType.kBrushless);
    private CANSparkMax rMotor1 = new CANSparkMax(4, MotorType.kBrushless);
    private CANSparkMax rMotor2 = new CANSparkMax(5, MotorType.kBrushless);

    //PID Controllers
    private CANPIDController lPidController = lMotor0.getPIDController();
    private CANPIDController rPidController = rMotor0.getPIDController();

    //MAX Encoders
    private CANEncoder lEncoder = lMotor0.getEncoder();
	private CANEncoder rEncoder = rMotor0.getEncoder();
	
	//Differential Drivetrain
	private DifferentialDrive drive = new DifferentialDrive(lMotor0, rMotor0);

    //PID Coefficients
	private double kMinOutput;
	private double kMaxOutput; 

	private double lKP;
	private double lKI; 
	private double lKIS;
	private double lKD;
	private double lKFF;

	private double rKP;
	private double rKI; 
	private double rKIS;
	private double rKD;
	private double rKFF;
	
	// Encoders for the motors
	private Encoder leftEncoder = new Encoder(RobotMap.p_leftEncoderA, RobotMap.p_leftEncoderB, true,
			EncodingType.k4X);
	private Encoder rightEncoder = new Encoder(RobotMap.p_rightEncoderA, RobotMap.p_rightEncoderB, true,
			EncodingType.k4X);

	// The NavX gyro
	private AHRS navX = new AHRS(SPI.Port.kMXP);
	
	// Initialize your subsystem here
	public DriveTrain() {
		registerCommands();
		
		//setting up follow mode!
		lMotor1.follow(lMotor0);
		lMotor2.follow(lMotor0);
		rMotor1.follow(rMotor0);
		rMotor2.follow(rMotor0);

		//setting up PID coefficients!
		lPidController.setP(lKP);
		lPidController.setI(lKI);
		lPidController.setD(lKD);
		lPidController.setIZone(lKIS);
		lPidController.setFF(lKFF);
		lPidController.setOutputRange(kMinOutput, kMaxOutput);

		rPidController.setP(rKP);
		rPidController.setI(rKI);
		rPidController.setD(rKD);
		rPidController.setIZone(rKIS);
		rPidController.setFF(rKFF);
		rPidController.setOutputRange(kMinOutput, kMaxOutput);



	}

	// Instantiate odometer and link in encoders and navX
	public Odometer odometer = new Odometer(0, 0, 0) {
		public void updateEncodersAndHeading() {

			this.headingAngle = 450 - navX.getFusedHeading();
			if(this.headingAngle > 360)
				this.headingAngle -= 360;

			this.leftPos = leftEncoder.getDistance();
			this.rightPos = rightEncoder.getDistance();

			this.currentVelocity = 0.5 * (leftEncoder.getRate() + rightEncoder.getRate());
		}
	};

	// Instantiate point controller for autonomous driving
	public DrivingController drivingcontroller = new DrivingController(0.0005) {

		// Use output from odometer and pass into autonomous driving controller
		@Override
		public void updateVariables() {
			this.currentX = odometer.current_x;
			this.currentY = odometer.current_y;
			this.currentAverageVelocity = odometer.currentVelocity;

			this.currentAngle = Robot.drivetrain.odometer.headingAngle;
		}

		// Link autonomous driving controller to the drive train motor control
		@Override
		public void driveRobot(double power, double pivot) {
			drive.arcadeDrive(power, pivot, false);
		}
	};

	// Subsystem run function, use controller collection (multi-threaded at fast
	// period)
	@Override
	public void run() {

		// Run every time
		this.odometer.integratePosition();

		// Run only when subsystem is enabled
		if (this.enabled) {
			this.drivingcontroller.run();
		}
	}

	// General arcade drive
	public void arcadeDrive(double power, double pivot) {
		drive.arcadeDrive(power, pivot);
	}

	// Sets drive train to 0
	public void drivetrainDisable() {
		lMotor0.setIdleMode(CANSparkMax.IdleMode.kBrake);
		rMotor0.setIdleMode(CANSparkMax.IdleMode.kBrake);

		lMotor0.set(0);
		rMotor0.set(0);
	}

	public void drivetrainEnable() {
		lMotor0.setIdleMode(CANSparkMax.IdleMode.kCoast);
		rMotor0.setIdleMode(CANSparkMax.IdleMode.kCoast);
	}

	@Override
	public void registerCommands() {
		new SubsystemCommand(this.registeredCommands, "set_angular_offset") {

			@Override
			public void initialize() {
				Robot.drivetrain.odometer.startOffset = Double.parseDouble(this.args[0]);
				Robot.drivetrain.navX.zeroYaw();
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

		new SubsystemCommand(this.registeredCommands, "add_forwards_spline") {

			@Override
			public void initialize() {
				Robot.drivetrain.drivingcontroller.addSpline(Double.parseDouble(this.args[0]),
						Double.parseDouble(this.args[1]), Double.parseDouble(this.args[2]),
						Double.parseDouble(this.args[3]), Double.parseDouble(this.args[4]),
						Double.parseDouble(this.args[5]), Double.parseDouble(this.args[6]),
						Double.parseDouble(this.args[7]), Double.parseDouble(this.args[8]),
						Double.parseDouble(this.args[9]), Double.parseDouble(this.args[10]),
						Double.parseDouble(this.args[11]), true);
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

		new SubsystemCommand(this.registeredCommands, "add_backwards_spline") {

			@Override
			public void initialize() {
				Robot.drivetrain.drivingcontroller.addSpline(Double.parseDouble(this.args[0]),
						Double.parseDouble(this.args[1]), Double.parseDouble(this.args[2]),
						Double.parseDouble(this.args[3]), Double.parseDouble(this.args[4]),
						Double.parseDouble(this.args[5]), Double.parseDouble(this.args[6]),
						Double.parseDouble(this.args[7]), Double.parseDouble(this.args[8]),
						Double.parseDouble(this.args[9]), Double.parseDouble(this.args[10]),
						Double.parseDouble(this.args[11]), false);
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

		new SubsystemCommand(this.registeredCommands, "start_path") {

			@Override
			public void initialize() {
				Robot.drivetrain.enabled = true;
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

}