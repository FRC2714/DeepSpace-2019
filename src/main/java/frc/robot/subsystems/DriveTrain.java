package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.util.DrivingController;
import frc.robot.util.Odometer;
import frc.robot.util.SubsystemCommand;
import frc.robot.util.SubsystemModule;

public class DriveTrain extends SubsystemModule {

	// Drivetrain motors
	private CANSparkMax lMotor0 = new CANSparkMax(0, MotorType.kBrushless);
	private CANSparkMax lMotor1 = new CANSparkMax(1, MotorType.kBrushless);
	private CANSparkMax lMotor2 = new CANSparkMax(2, MotorType.kBrushless);
	private CANSparkMax rMotor0 = new CANSparkMax(3, MotorType.kBrushless);
	private CANSparkMax rMotor1 = new CANSparkMax(4, MotorType.kBrushless);
	private CANSparkMax rMotor2 = new CANSparkMax(5, MotorType.kBrushless);

	// PID controllers
	private CANPIDController lPidController = lMotor0.getPIDController();
	private CANPIDController rPidController = rMotor0.getPIDController();

	// MAX encoders
	private CANEncoder lEncoder = lMotor0.getEncoder();
	private CANEncoder rEncoder = rMotor0.getEncoder();

	// Differential drivetrain
	private DifferentialDrive drive = new DifferentialDrive(lMotor0, rMotor0);

	// PID coefficients
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

	private int debugMode;

	// Robot characteristics
	private double wheelSeparation = 2;

	// Gearbox encoders
	private Encoder leftEncoder = new Encoder(RobotMap.p_leftEncoderA, RobotMap.p_leftEncoderB, true, EncodingType.k4X);
	private Encoder rightEncoder = new Encoder(RobotMap.p_rightEncoderA, RobotMap.p_rightEncoderB, true,
			EncodingType.k4X);

	// NavX gyro
	private AHRS navX = new AHRS(SPI.Port.kMXP);

	// Drivetrain initialization
	public DriveTrain() {
		registerCommands();

		// Configure follow mode
		lMotor1.follow(lMotor0);
		lMotor2.follow(lMotor0);
		rMotor1.follow(rMotor0);
		rMotor2.follow(rMotor0);

		// Setup up PID coefficients
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

		// SmartDashboard configuration
		SmartDashboard.putNumber("Left P Gain", lKP);
		SmartDashboard.putNumber("Left I Gain", lKI);
		SmartDashboard.putNumber("Left D Gain", lKD);
		SmartDashboard.putNumber("Left I Zone", lKIS);
		SmartDashboard.putNumber("Left Feed Forward", lKFF);

		SmartDashboard.putNumber("Right P Gain", rKP);
		SmartDashboard.putNumber("Right I Gain", rKI);
		SmartDashboard.putNumber("Right D Gain", rKD);
		SmartDashboard.putNumber("Right I Zone", rKIS);
		SmartDashboard.putNumber("Right Feed Forward", rKFF);

		SmartDashboard.putNumber("Max Output", kMaxOutput);
		SmartDashboard.putNumber("Min Output", kMinOutput);
		SmartDashboard.putNumber("Debug Mode", debugMode);
	}

	// Instantiate odometer and link in encoders and navX
	public Odometer odometer = new Odometer(0,0,0) {

		@Override
		public void updateEncodersAndHeading() {
			this.headingAngle = 450 - navX.getFusedHeading();
			if(this.headingAngle>360) {
				this.headingAngle-=360;
			}	

			this.leftPos=leftEncoder.getDistance();
			this.rightPos=rightEncoder.getDistance();

			this.currentAverageVelocity = (leftEncoder.getRate() + rightEncoder.getRate()) / 2;
	
		}

	};

	// Instantiate point controller for autonomous driving
	public DrivingController drivingcontroller = new DrivingController(0.0005) {

		// Use output from odometer and pass into autonomous driving controller
		@Override
		public void updateVariables(){
			this.currentX = odometer.getCurrentX();
			this.currentY = odometer.getCurrentY();
			this.currentAngle = odometer.getHeadingAngle();
			this.currentAverageVelocity = odometer.getCurrentAverageVelocity();
		}

		// Link autonomous driving controller to the drive train motor control
		@Override
		public void driveRobot(double power,double pivot) {
			drive.arcadeDrive(power,pivot,false);
		}
	};

	@Override
	public void init() {
		leftEncoder.reset();
		rightEncoder.reset();
		navX.reset();
		navX.zeroYaw();

		leftEncoder.setDistancePerPulse(-0.0495);
		rightEncoder.setDistancePerPulse(0.00105);

		lMotor0.setIdleMode(CANSparkMax.IdleMode.kCoast);
		rMotor0.setIdleMode(CANSparkMax.IdleMode.kCoast);
	}

	@Override
	public void destruct() {
		lMotor0.setIdleMode(CANSparkMax.IdleMode.kBrake);
		rMotor0.setIdleMode(CANSparkMax.IdleMode.kBrake);

		lMotor0.set(0);
		rMotor0.set(0);
	}
	
	// Subsystem run function, use controller collection (multi-threaded at fast period)
	@Override
	public void run() {

		// Run every time
		this.odometer.integratePosition();

		// Run only when subsystem is enabled
		if (this.enabled) {
			this.drivingcontroller.run();
		}
	}

	// Pull values from SmartDashboard
	public void configureCoefficients() {
		double lP = SmartDashboard.getNumber("Left P Gain", 0);
		double lI = SmartDashboard.getNumber("Left I Gain", 0);
		double lD = SmartDashboard.getNumber("Left D Gain", 0);
		double lIS = SmartDashboard.getNumber("Left I Saturation", 0);
		double lFF = SmartDashboard.getNumber("Left Feed Forward", 0);

		double rP = SmartDashboard.getNumber("Right P Gain", 0);
		double rI = SmartDashboard.getNumber("Right I Gain", 0);
		double rD = SmartDashboard.getNumber("Right D Gain", 0);
		double rIS = SmartDashboard.getNumber("Right I Saturation", 0);
		double rFF = SmartDashboard.getNumber("Right Feed Forward", 0);
						
		double max = SmartDashboard.getNumber("Max Output", 0);
		double min = SmartDashboard.getNumber("Min Output", 0);
	
		// If PID coefficients on SmartDashboard have changed, write new values to controller
		if (lP != lKP) { lPidController.setP(lP); lKP = lP; }						
		if (lI != lKI) { lPidController.setI(lI); lKI = lI; }
		if (lD != lKD) { lPidController.setD(lD); lKD = lD; }			
		if (lIS != lKIS) { lPidController.setIZone(lIS); lKIS = lIS; }
		if (lFF != lKFF) { lPidController.setFF(lFF); lKFF = lFF; }

		if (rP != rKP) { rPidController.setP(rP); rKP = rP; }
		if (rI != rKI) { rPidController.setI(rI); rKI = rI; }
		if (rD != rKD) { rPidController.setD(rD); rKD = rD; }
		if (rIS != rKIS) { rPidController.setIZone(rIS); rKIS = rIS;}
		if (rFF != rKFF) { rPidController.setFF(rFF); rKFF = rFF; }
						
		if ((max != kMaxOutput) || (min != kMinOutput)) { 
			lPidController.setOutputRange(min, max); 
			kMinOutput = min; kMaxOutput = max;
		}
	}

	// General arcade drive
	public void arcadeDrive(double power, double pivot) {
		drive.arcadeDrive(power, pivot);
	}

	// Closed loop velocity based tank
	public void closedLoopTank(double leftVelocity, double rightVelocity) {
		
		if(SmartDashboard.getNumber("DebugMode", 0) > 0){
			configureCoefficients();																			
		}
				
		lPidController.setReference(leftVelocity, ControlType.kVelocity);
		rPidController.setReference(rightVelocity, ControlType.kVelocity);
	}

	// Closed loop arcade based tank
	public void closedLoopArcade(double velocity, double rps) {
		double pivot = Math.PI * wheelSeparation * rps;
		closedLoopTank(velocity - pivot, velocity + pivot);
	}

	// Output encoder values
	public void getEncoderValues() {
		System.out.println("LE: " + lEncoder.getPosition() + " RE: " + rEncoder.getPosition());
	}

	@Override
	public void registerCommands() {
		new SubsystemCommand(this.registeredCommands, "set_angular_offset") {

			@Override
			public void initialize() {
				odometer.setOffset(Double.parseDouble(this.args[0]));
				navX.zeroYaw();
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
				drivingcontroller.addSpline(Double.parseDouble(this.args[0]), Double.parseDouble(this.args[1]),
						Double.parseDouble(this.args[2]), Double.parseDouble(this.args[3]),
						Double.parseDouble(this.args[4]), Double.parseDouble(this.args[5]),
						Double.parseDouble(this.args[6]), Double.parseDouble(this.args[7]),
						Double.parseDouble(this.args[8]), Double.parseDouble(this.args[9]),
						Double.parseDouble(this.args[10]), Double.parseDouble(this.args[11]), true);
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
				drivingcontroller.addSpline(Double.parseDouble(this.args[0]), Double.parseDouble(this.args[1]),
						Double.parseDouble(this.args[2]), Double.parseDouble(this.args[3]),
						Double.parseDouble(this.args[4]), Double.parseDouble(this.args[5]),
						Double.parseDouble(this.args[6]), Double.parseDouble(this.args[7]),
						Double.parseDouble(this.args[8]), Double.parseDouble(this.args[9]),
						Double.parseDouble(this.args[10]), Double.parseDouble(this.args[11]), false);
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
				enabled = true;
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