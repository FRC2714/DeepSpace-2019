package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.util.ControlsProcessor;
import frc.robot.util.DrivingController;
import frc.robot.util.Odometer;
import frc.robot.util.SubsystemCommand;
import frc.robot.util.SubsystemModule;

@SuppressWarnings("Duplicates")
public class DriveTrain extends SubsystemModule {

	// Drivetrain motors
	private CANSparkMax lMotor0 = new CANSparkMax(1, MotorType.kBrushless);
	private CANSparkMax lMotor1 = new CANSparkMax(2, MotorType.kBrushless);
	private CANSparkMax lMotor2 = new CANSparkMax(3, MotorType.kBrushless);
	private CANSparkMax rMotor0 = new CANSparkMax(4, MotorType.kBrushless);
	private CANSparkMax rMotor1 = new CANSparkMax(5, MotorType.kBrushless);
	private CANSparkMax rMotor2 = new CANSparkMax(6, MotorType.kBrushless);

	// PID controllers
	private CANPIDController lPidController = lMotor0.getPIDController();
	private CANPIDController rPidController = rMotor0.getPIDController();

	// MAX encoders
	private CANEncoder lEncoder = lMotor0.getEncoder();
	private CANEncoder rEncoder = rMotor0.getEncoder();

	// Differential drivetrain
	private DifferentialDrive drive = new DifferentialDrive(lMotor0, rMotor0);

	// PID coefficients
	private final double kMinOutput = -1;
	private final double kMaxOutput = 1;

	private final double kP = 4.8e-5;
	private final double kI = 5.0e-7;
	private final double kD = 0.0;
	private final double kIS = 0.0;

	private final double lKFF = 1.77e-4;
	private final double rKFF = 1.78e-4;

	private final double rpmToFeet = 0.003135; // Convert RPM to ft/s
	private final double rotationsToFeet = 0.1881; // Convert rotations to feet

	private double startTime;
	private int numberOfRuns;


	private final double sensitivity = 2.5;
	private final double maxVelocity = 13;
	private final double maxAcceleration = 5;

	private double collisionThreshold = 0;
	private double tippingThreshold = 0;

	private double prevAccelX = 0;
	private double prevAccelY = 0;
	private double mPrevTimeAccel = 0;

	// 
	private double leftEncoderOffset = 0;
	private double rightEncoderOffset = 0;
	private double lastVelocity = 0;

	// Ramp code
	private double currentOpenArcadePower;
	private double currentOpenArcadePivot;

	// Robot characteristics
	private double wheelSeparation = 2;

	private boolean driverControlled = false;

	private ControlsProcessor controlsProcessor;

	// Gearbox encoders
	private Encoder leftShaftEncoder = new Encoder(RobotMap.p_leftEncoderA, RobotMap.p_leftEncoderB, true, EncodingType.k4X);
	private Encoder rightShaftEncoder = new Encoder(RobotMap.p_rightEncoderA, RobotMap.p_rightEncoderB, true,
			EncodingType.k4X);

	// NavX gyro
	private AHRS navX = new AHRS(SPI.Port.kMXP);

	//limelight
	NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

	// Drivetrain initializations
	public DriveTrain(ControlsProcessor controlsProcessor) {
		registerCommands();

		this.controlsProcessor = controlsProcessor;

		drive.setSafetyEnabled(false);
		// Configure follow mode
		lMotor1.follow(lMotor0);
		lMotor2.follow(lMotor0);
		rMotor1.follow(rMotor0);
		rMotor2.follow(rMotor0);

		// Setup up PID coefficients
		lPidController.setP(kP);
		lPidController.setI(kI);
		lPidController.setD(kD);
		lPidController.setIZone(kIS);
		lPidController.setFF(lKFF);
		lPidController.setOutputRange(kMinOutput, kMaxOutput);

		rPidController.setP(kP);
		rPidController.setI(kI);
		rPidController.setD(kD);
		rPidController.setIZone(kIS);
		rPidController.setFF(rKFF);
		rPidController.setOutputRange(kMinOutput, kMaxOutput);
	}

	// Instantiate odometer and link in encoders and navX
	public Odometer odometer = new Odometer(0,0,0) {

		@Override
		public void updateEncodersAndHeading() {
			this.headingAngle = -navX.getYaw() + 90;
			if(this.headingAngle < 0) {
				this.headingAngle += 360;
			}	

			this.leftPos = leftShaftEncoder.getDistance();
			this.rightPos = rightShaftEncoder.getDistance();
			
			double leftVelocity = leftShaftEncoder.getRate();
			double rightVelocity = rightShaftEncoder.getRate();

			this.currentAverageVelocity = (leftVelocity + rightVelocity) / 2;	
		}
	};

	// Instantiate point controller for autonomous driving
	public DrivingController drivingController = new DrivingController(0.01) {

		/**
		 * Use output from odometer and pass into autonomous driving controller
		 */
		@Override
		public void updateVariables(){
			this.currentX = odometer.getCurrentX();
			this.currentY = odometer.getCurrentY();
			this.currentAngle = odometer.getHeadingAngle();
			this.currentAverageVelocity = odometer.getCurrentAverageVelocity();
		}

		/**
		 * Link autonomous driving controller to the drive train motor control
		 */
		@Override
		public void driveRobot(double power, double pivot) {
			closedLoopArcade(power, pivot);
		}
	};

	/**
	 * Resets the variables for the drivetrain
	 */
	@Override
	public void init() {
		System.out.println("resetting");
		leftEncoderOffset = lEncoder.getPosition();
		rightEncoderOffset = -rEncoder.getPosition();
		navX.reset();
		navX.zeroYaw();

		currentOpenArcadePower = 0;
		currentOpenArcadePivot = 0;

		// leftEncoder.setDistancePerPulse(-0.0495);
		// rightEncoder.setDistancePerPulse(0.00105);
		leftShaftEncoder.reset();
		rightShaftEncoder.reset();
		leftShaftEncoder.setDistancePerPulse(0.0007819);
		rightShaftEncoder.setDistancePerPulse(0.00078012);

		lMotor0.setIdleMode(CANSparkMax.IdleMode.kCoast);
		rMotor0.setIdleMode(CANSparkMax.IdleMode.kCoast);
	}

	/**
	 * Disables the motors and stops the drivetrain
	 */
	@Override
	public void destruct() {
		driverControlled = false;

		lMotor0.setIdleMode(CANSparkMax.IdleMode.kBrake);
		rMotor0.setIdleMode(CANSparkMax.IdleMode.kBrake);

		lMotor0.set(0);
		rMotor0.set(0);

		disable();
		drivingController.clearControlPath();
	}
	
	/**
	 * Subsystem run function, uses ControlsProcessor (multi-threaded at fast period)
	 */
	@Override
	public void run() {

		// Run every time
		this.odometer.integratePosition();

		// Run only when subsystem is enabled
		if (getStatus()) {
			this.drivingController.run();
		}
	}

	// General arcade drive
	public void arcadeDrive(double power, double pivot) {
		drive.arcadeDrive(power, pivot);
	}

	public void arcadeDrive(double power, double pivot, double rampUp, double rampDown) {
		int currentDirection = (int)(Math.abs(currentOpenArcadePower) / currentOpenArcadePower);
		int desiredDirection = (int)(Math.abs(power) / power);

		if (currentDirection * desiredDirection > 0) {
			if(currentOpenArcadePower < power) {
				currentOpenArcadePower += rampUp;
				
				if(currentOpenArcadePower > power) { currentOpenArcadePower = power; }
			}
			else if(currentOpenArcadePower > power) {
				currentOpenArcadePower -= rampUp;

				if(currentOpenArcadePower < power) { currentOpenArcadePower = power; }
			}
		} else {
			if(currentOpenArcadePower < power) {
				currentOpenArcadePower += rampDown;
				
				if(currentOpenArcadePower > power) { currentOpenArcadePower = power; }
			}
			else if(currentOpenArcadePower > power) {
				currentOpenArcadePower -= rampDown;

				if(currentOpenArcadePower < power) { currentOpenArcadePower = power; }
			}
		}

		// System.out.println("Current Arcade Power: " + currentOpenArcadePower + "\tCurrent Arcade Pivot: " + currentOpenArcadePivot);
		arcadeDrive(currentOpenArcadePower, pivot);
	}

	// Closed loop velocity based tank without an acceleration limit
	public void closedLoopTank(double leftVelocity, double rightVelocity) {
		lPidController.setReference(leftVelocity / rpmToFeet, ControlType.kVelocity);
		rPidController.setReference(-rightVelocity / rpmToFeet, ControlType.kVelocity);
		//System.out.println("ls: " + leftVelocity / rpmToFeet + " rs: " + -rightVelocity / rpmToFeet);
	}

	public synchronized void setCollisionJerkThreshold(double jerkCollisionThreshold) {
		collisionThreshold = jerkCollisionThreshold;
	}

	public synchronized void setTippingThreshold(double tippingThreshold) {
		this.tippingThreshold = tippingThreshold;
	}


	public boolean isTipping() {
		return Math.abs(navX.getPitch()) > tippingThreshold ||
				Math.abs(navX.getRoll()) > tippingThreshold;
	}

	public boolean isCollisionOccurring() {
		boolean collisionOccurring = false;

		double accelX = navX.getWorldLinearAccelX();
		double accelY = navX.getWorldLinearAccelY();


		double currTime = Timer.getFPGATimestamp();
		double dt = currTime - mPrevTimeAccel;

		double jerkX = (accelX - prevAccelX) / (dt);
		double jerkY = (accelY - prevAccelY) / (dt);

		if (Math.abs(jerkX) > collisionThreshold || Math.abs(jerkY) > collisionThreshold)
			collisionOccurring = true;

		prevAccelX = accelX;
		prevAccelY = accelY;

		if (mPrevTimeAccel == 0) {
			mPrevTimeAccel = currTime;
			return false;
		}

		mPrevTimeAccel = currTime;
		return collisionOccurring;
	}

	// Closed loop arcade based tank
	public void closedLoopArcade(double velocity, double pivot) {
		pivot = pivot * sensitivity;
		closedLoopTank(velocity - pivot, velocity + pivot);
		//System.out.println("pivot " + pivot);
	}

	// Closed loop velocity based tank with an acceleration limit
	public void closedLoopArcade(double velocity, double pivot, double accelLimit) {
		accelLimit *= controlsProcessor.getCommandPeriod();

		double velocitySetpoint = velocity;

		if (Math.abs(velocity - lastVelocity) > accelLimit) {
			if (velocity - lastVelocity > 0)
				velocitySetpoint = lastVelocity + accelLimit;
			else
				velocitySetpoint = lastVelocity - accelLimit;
		}

		closedLoopArcade(velocitySetpoint, pivot);

		lastVelocity = velocitySetpoint;
		System.out.println("velocity setpoint: " + velocitySetpoint);
	}

	// Output encoder values
	public void getEncoderValues() {
		System.out.println("LE: " + leftShaftEncoder.getDistance() + " RE: " + rightShaftEncoder.getDistance());
	}

	public double getMaxVelocity(){
		return maxVelocity;
	}

	@Override
	public void registerCommands() {
		new SubsystemCommand(this.registeredCommands, "driver_control") {

			@Override
			public void initialize() {
				driverControlled = true;
				// System.out.println("Right Encoder: " + rightShaftEncoder.getDistance() + "\tLeft Encoder: " + leftShaftEncoder.getDistance());
			}

			@Override
			public void execute() {
				double power = 0;
				double pivot = 0;

				if (Math.abs(controlsProcessor.getLeftJoystick()) > .15)	
					power = controlsProcessor.getLeftJoystick();
				if (Math.abs(controlsProcessor.getRightJoystick()) > .15)
					pivot = controlsProcessor.getRightJoystick();

				arcadeDrive(-power, pivot, 0.04, 0.08);

				//System.out.println("Odometer heading angle " + odometer.getHeadingAngle());
			}

			@Override
			public boolean isFinished() {
				return !driverControlled;
			}

			@Override
			public void end() {
				closedLoopArcade(0, 0);
			}
		};

		new SubsystemCommand(this.registeredCommands, "closed_loop_tank") {

			@Override
			public void initialize() {
				driverControlled = false;

				double velocity = Double.parseDouble(this.args[0]);
				closedLoopTank(velocity, velocity);
			}

			@Override
			public void execute() {

			}

			@Override
			public boolean isFinished() {
				return false;
			}

			@Override
			public void end() {
				closedLoopTank(0, 0);
			}
		};

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

		new SubsystemCommand(this.registeredCommands, "debug_print") {

			@Override
			public void initialize() {

				lMotor0.setIdleMode(CANSparkMax.IdleMode.kCoast);
				rMotor0.setIdleMode(CANSparkMax.IdleMode.kCoast);

				lMotor0.set(0);
				rMotor0.set(0);
				
			}

			@Override
			public void execute() {
				getEncoderValues();
				//System.out.println(odometer.getHeadingAngle());
				//System.out.println(odometer.getCurrentX() + " : " + odometer.getCurrentY());
				//System.out.println(navX.getYaw());
			}

			@Override
			public boolean isFinished() {
				return false;
			}

			@Override
			public void end() {

			}
		};
		new SubsystemCommand(this.registeredCommands, "add_forwards_spline") {

			@Override
			public void initialize() {

				double xInitial = Double.parseDouble(this.args[0]);
				double xFinal = Double.parseDouble(this.args[4]);

				double yInitial = Double.parseDouble(this.args[1]);
				double yFinal = Double.parseDouble(this.args[5]);

				double thetaInitial = Double.parseDouble(this.args[2]);
				double thetaFinal = Double.parseDouble(this.args[6]);

				double lInitial = Double.parseDouble(this.args[3]);
				double lFinal = Double.parseDouble(this.args[7]);

				thetaInitial = Math.toRadians(thetaInitial);
				thetaFinal = Math.toRadians(thetaFinal);

				double x2 = lInitial * Math.cos(thetaInitial) + xInitial;
				double x3 = lFinal * Math.cos(thetaFinal + Math.PI) + xFinal; 
				double y2 = lInitial * Math.sin(thetaInitial) + yInitial; 
				double y3 = lFinal * Math.sin(thetaFinal + Math.PI) + yFinal; 

				drivingController.addSpline(xInitial, x2, x3, xFinal, yInitial, y2, y3, yFinal,
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


		new SubsystemCommand(this.registeredCommands, "add_forwards_spline_dynamic") {

			@Override
			public void initialize() {

				double xInitial = odometer.getCurrentX();
				double xFinal = Double.parseDouble(this.args[1]);

				double yInitial = odometer.getCurrentY();
				double yFinal = Double.parseDouble(this.args[2]);

				double thetaInitial = odometer.getHeadingAngle();
				double thetaFinal = Double.parseDouble(this.args[3]);

				double lInitial = Double.parseDouble(this.args[0]);
				double lFinal = Double.parseDouble(this.args[4]);

				thetaInitial = Math.toRadians(thetaInitial);
				thetaFinal = Math.toRadians(thetaFinal);

				double x2 = lInitial * Math.cos(thetaInitial) + xInitial;
				double x3 = lFinal * Math.cos(thetaFinal + Math.PI) + xFinal;
				double y2 = lInitial * Math.sin(thetaInitial) + yInitial;
				double y3 = lFinal * Math.sin(thetaFinal + Math.PI) + yFinal;

				drivingController.addSpline(xInitial, x2, x3, xFinal, yInitial, y2, y3, yFinal,
						Double.parseDouble(this.args[5]), Double.parseDouble(this.args[6]),
						Double.parseDouble(this.args[7]), Double.parseDouble(this.args[8]), true);
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

				double xInitial = Double.parseDouble(this.args[0]);
				double yInitial = Double.parseDouble(this.args[1]);
				double thetaInitial = Double.parseDouble(this.args[2]);
				double lInitial = Double.parseDouble(this.args[3]);
				double xFinal = Double.parseDouble(this.args[4]);
				double yFinal = Double.parseDouble(this.args[5]);
				double thetaFinal = Double.parseDouble(this.args[6]);
				double lFinal = Double.parseDouble(this.args[7]);

				thetaInitial = Math.toRadians(thetaInitial);
				thetaFinal = Math.toRadians(thetaFinal);

				double x2 = lInitial * Math.cos(thetaInitial + Math.PI) + xInitial;
				double x3 = lFinal * Math.cos(thetaFinal) + xFinal; 
				double y2 = lInitial * Math.sin(thetaInitial + Math.PI) + yInitial; 
				double y3 = lFinal * Math.sin(thetaFinal) + yFinal; 

				
				drivingController.addSpline(xInitial, x2, x3, xFinal, yInitial, y2, y3, yFinal,
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

		new SubsystemCommand(this.registeredCommands, "add_backwards_spline_dynamic") {

			@Override
			public void initialize() {

				System.out.println("Current X : " + odometer.getCurrentX() + " || Current Y : " + odometer.getCurrentY());
				System.out.println("DYNAMIC HEADING ANGLE:- " + odometer.getHeadingAngle());

				double xInitial = odometer.getCurrentX();
				double xFinal = Double.parseDouble(this.args[1]);

				double yInitial = odometer.getCurrentY();
				double yFinal = Double.parseDouble(this.args[2]);

				double thetaInitial = odometer.getHeadingAngle();
				double thetaFinal = Double.parseDouble(this.args[3]);
				System.out.println("Theta Final:- " + this.args[3]);

				double lInitial = Double.parseDouble(this.args[0]);
				double lFinal = Double.parseDouble(this.args[4]);

				thetaInitial = Math.toRadians(thetaInitial);
				thetaFinal = Math.toRadians(thetaFinal);

				double x2 = lInitial * Math.cos(thetaInitial) + xInitial;
				double x3 = lFinal * Math.cos(thetaFinal + Math.PI) + xFinal;
				double y2 = lInitial * Math.sin(thetaInitial) + yInitial;
				double y3 = lFinal * Math.sin(thetaFinal + Math.PI) + yFinal;

				drivingController.addSpline(xInitial, x2, x3, xFinal, yInitial, y2, y3, yFinal,
						Double.parseDouble(this.args[5]), Double.parseDouble(this.args[6]),
						Double.parseDouble(this.args[7]), Double.parseDouble(this.args[8]), false);
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
			double startTime;
			int counter = 0;

			@Override
			public void initialize() {
				drivingController.setIsFinished(false);
				enable();
				System.out.println("starting path");
			}

			@Override
			public void execute() {
				if(counter < 5){
					startTime = System.nanoTime();
				}
//				System.out.println("CURRENT HEADING ANGLE: " + odometer.getHeadingAngle());
//				double averageTime = (System.nanoTime() - startTime)/counter;

				//System.out.println("average time " + averageTime);
				counter++;
			}

			@Override
			public boolean isFinished() {
//				System.out.println("driving controller finished? " + ":" + drivingController.isFinished());
				return drivingController.isFinished();
			}

			@Override
			public void end() {
				closedLoopArcade(0, 0);
				disable();
				System.out.println(odometer.getCurrentX() + " : " + odometer.getCurrentY() + "Final Heading : " + odometer.getHeadingAngle());
			}
		};

		new SubsystemCommand(this.registeredCommands, "delay_tester"){
			@Override
			public void initialize() {
//				System.out.println("Delay = " + Double.parseDouble(this.args[1]));
//				enable();
			}

			@Override
			public void execute() {
				System.out.println("DELAY TESTER running!" );
			}

			@Override
			public boolean isFinished() {
				return false;
			}

			@Override
			public void end() {
			}
		};

		new SubsystemCommand(this.registeredCommands, "drive_to_target"){
			double initialX;
			double initialY;
			double theta1;

			@Override
			public void initialize() {

				lMotor0.setIdleMode(IdleMode.kCoast);
				rMotor0.setIdleMode(IdleMode.kCoast); 

				lMotor0.set(0);    
				rMotor0.set(0);


				initialX = odometer.getCurrentX();
				initialY = odometer.getCurrentY();

				theta1 = odometer.getHeadingAngle() - limelightTable.getEntry("tx").getDouble(0);

				if (theta1 > 360)
					theta1 -= 360;
				else if (theta1 < 0)
					theta1 += 360;

				// //Stage 1	
				// 	double currentX = odometer.getCurrentX();
				// 	double currentY = odometer.getCurrentY();
				// 	double odometerHeading = odometer.getHeadingAngle();

				// 	double limelightX = 1.25*Math.cos(Math.toRadians(odometer.getHeadingAngle()));
				// 	double limelightY = -1.25*Math.sin(Math.toRadians(odometer.getHeadingAngle()));

				// 	System.out.println(currentX + " : " + currentY + " : " + limelightX + " : " + limelightY);

				// //Stage 2
				// 	NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
				// 	double centerAngle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
				// 	NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
				// 	double rightAngle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
				// 	double angleDifference = rightAngle - centerAngle;

				// NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
				// NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);

				// double ySubtraction = Double.parseDouble(this.args[0]);

				// NetworkTableEntry camtran = table.getEntry("camtran");
				// // double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
				// // double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
				// // double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
				// // double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

				// double data[] = new double[6];

				// data = camtran.getDoubleArray(data);
				// System.out.println(data[1] + " : " + data[2] + " : " + data[4]);

				// if(data[1] == 0){
				// 	this.cancel();
				// 	return;
				// }

				// double thetaInitial = Math.toRadians(odometer.getHeadingAngle());
				// double thetaFinal = Math.toRadians(-data[4] + odometer.getHeadingAngle());
				// double lInitial = 1;
				// double lFinal = 1;

				// double limelightX = data[1]/12;
				// double limelightY = data[2]/12;
				// double customY = limelightY + ySubtraction;

				// double customHypotenuse = Math.sqrt(Math.pow(customY, 2) + Math.pow(limelightX, 2));

				// double customTheta = Math.atan(limelightX/customY);


				// double xFinal = customHypotenuse*Math.cos(Math.toRadians(odometer.getHeadingAngle()) + customTheta) + odometer.getCurrentX() + cameraXOffset;
				// double yFinal = customHypotenuse*Math.sin(Math.toRadians(odometer.getHeadingAngle()) + customTheta) + odometer.getCurrentY() + cameraYOffset;

				// double xInitial = odometer.getCurrentX();
				// double yInitial = odometer.getCurrentY();

				// double x2 = lInitial * Math.cos(thetaInitial) + odometer.getCurrentX();
				// double x3 = lFinal * Math.cos(thetaFinal + Math.PI) + xFinal;
				// double y2 = lInitial * Math.sin(thetaInitial) + odometer.getCurrentY();
				// double y3 = lFinal * Math.sin(thetaFinal + Math.PI) + yFinal;

				//System.out.println(xInitial + " : " + yInitial + " : " + odometer.getHeadingAngle() + " : " + xFinal + " : " + yFinal + " : " + (-data[4] + odometer.getHeadingAngle()));
				
				// drivingController.addSpline(xInitial, x2, x3, xFinal, yInitial, y2, y3, yFinal,
				//  		10, 4, 0, 0, true);
								
				// enable();
			}
			double l2;
			@Override
			public void execute() {
				double deltaX = odometer.getCurrentX() - initialX;
				double deltaY = odometer.getCurrentY() - initialY;

				double theta2 = odometer.getHeadingAngle() - limelightTable.getEntry("tx").getDouble(0);

				if (theta2 > 360)
					theta2 -= 360;
				else if (theta2 < 0)
					theta2 += 360;

				double num1 = deltaY*Math.cos(Math.toRadians(theta1));
				double num2 = deltaX*Math.sin(Math.toRadians(theta1));
				double denom1 = Math.sin(Math.toRadians(theta1))*Math.cos(Math.toRadians(theta2));
				double denom2 = Math.sin(Math.toRadians(theta2))*Math.cos(Math.toRadians(theta1));
				
				l2 =  (num1 - num2) / (denom1 - denom2);
				System.out.println(theta1 + " : " + theta2 + " ; " + deltaX + " : " + deltaY);
			}

			@Override
			public boolean isFinished() {
				return false;
			}

			@Override
			public void end() {
				System.out.println("l2: " + l2);
			}
		};


		new SubsystemCommand(this.registeredCommands, "wait") {

			Timer waitTimer = new Timer();
			@Override
			public void initialize() {
				waitTimer.reset();
				waitTimer.start();
				
			}

			@Override
			public void execute() {
			}

			@Override
			public boolean isFinished() {
				return waitTimer.get() > Double.parseDouble(this.args[0]);
			}

			@Override
			public void end() {
			}
		};

		new SubsystemCommand(this.registeredCommands, "vision_align"){
			@Override
			public void initialize() {
				// driverControlled = false;
				limelightTable.getEntry("ledMode").setNumber(3);
				limelightTable.getEntry("camMode").setNumber(0);

				System.out.println("initializing");
			}

			@Override
			public void execute() {
				double tx = limelightTable.getEntry("tx").getDouble(0);

				double kAngleP = 0.065;

				double power = 0;
				double pivot = tx * kAngleP;

				if (Math.abs(controlsProcessor.getLeftJoystick()) > .10)
					power = -controlsProcessor.getLeftJoystick();

				closedLoopArcade(power*(maxVelocity/2), -pivot);

//
//				if (tx > 0){
//					System.out.println("Turning Right Pivot: " + pivot);
//					closedLoopTank((power * maxVelocity) + pivot, (power * maxVelocity));
//				} else if (tx < 0){
//					System.out.println("Turning Left Pivot: " + pivot);
//					closedLoopTank((power * maxVelocity), (power * maxVelocity) - pivot);
//				}


			}

			@Override
			public boolean isFinished() {
				return false;
			}

			@Override
			public void end() {
				limelightTable.getEntry("camMode").setNumber(1);
				limelightTable.getEntry("ledMode").setNumber(1);
				closedLoopArcade(0, 0);
			}
		};


		new SubsystemCommand(this.registeredCommands, "auton_vision_align"){
			boolean isAboveMax = false;
			double maxBlobArea = 6.1;
			double currentBlobArea;

			@Override
			public void initialize() {
				limelightTable.getEntry("ledMode").setNumber(3);
				limelightTable.getEntry("camMode").setNumber(0);
				isAboveMax = false;
			}


			@Override
			public void execute() {
				System.out.println("Running");
				double tx = limelightTable.getEntry("tx").getDouble(0);
				currentBlobArea = limelightTable.getEntry("ta").getDouble(0);

				double kAngleP = 0.05;
				double kDistanceDivisor = 0.55; // Untested value. Direct proportionality.

				double power = 0;
				if (currentBlobArea < maxBlobArea && currentBlobArea != 0)
					power = kDistanceDivisor / currentBlobArea;
				else
					power = 0;

				double pivot = tx * kAngleP;

				System.out.println("kDistanceDivisor: " + kDistanceDivisor + "| blobArea : " + currentBlobArea);


				if (power > 0.4)
					power = 0.4;

				System.out.println("power: " + power);

				if (currentBlobArea <= maxBlobArea) {
					closedLoopArcade(power * maxVelocity, -pivot);
				}

				isAboveMax = currentBlobArea > maxBlobArea;
			}

			@Override
			public boolean isFinished() {
				System.out.println("Boolean : " + (currentBlobArea > maxBlobArea));
				return isAboveMax;
			}

			@Override
			public void end() {
				limelightTable.getEntry("camMode").setNumber(0);
				closedLoopArcade(0, 0);
				limelightTable.getEntry("ledMode").setNumber(1);
				System.out.println("x: " + odometer.getCurrentX() + "y: " + odometer.getCurrentY() + "thetaF: " + odometer.getHeadingAngle());
			}
		};

		new SubsystemCommand(this.registeredCommands, "set_current_position"){
			@Override
			public void initialize() {
				odometer.setCurrentPosition(Double.parseDouble(this.args[0]), Double.parseDouble(this.args[1]));
				System.out.println("SET POSITIONS: " + " X = " + odometer.getCurrentX() + " Y = " + odometer.getCurrentY());
			}

			@Override
			public boolean isFinished() {
				return true;
			}
		};
		
	}

}