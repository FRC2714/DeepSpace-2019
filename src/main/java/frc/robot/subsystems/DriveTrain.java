package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.util.*;

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

	private final double sensitivity = 2.5;
	private final double maxVelocity = 13;

	private double lastVelocity = 0;

	// Ramp code
	private double currentOpenArcadePower;

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

		lMotor0.enableVoltageCompensation(12.0);
		rMotor0.enableVoltageCompensation(12.0);

		lMotor0.setSmartCurrentLimit(50);
		lMotor1.setSmartCurrentLimit(50);
		lMotor2.setSmartCurrentLimit(50);

		rMotor0.setSmartCurrentLimit(50);
		rMotor1.setSmartCurrentLimit(50);
		rMotor2.setSmartCurrentLimit(50);

		drivingController.clearControlPath();
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
		navX.reset();
		navX.zeroYaw();
		
		odometer.reset();

		currentOpenArcadePower = 0;

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
		// System.out.println("ls: " + leftVelocity / rpmToFeet + " rs: " + -rightVelocity / rpmToFeet);
	}

	// Closed loop arcade based tank
	public void closedLoopArcade(double velocity, double pivot) {
		pivot = pivot * sensitivity;
		closedLoopTank(velocity - pivot, velocity + pivot);
		// System.out.println("pivot " + pivot);
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

	public void addForwardSpline(double xInitial, double yInitial, double thetaInitial, double lInitial,
			double xFinal, double yFinal, double thetaFinal, double lFinal, double maxAcceleration,
			double maxVelocity, double startVelocity, double endVelocity) {

		thetaInitial = Math.toRadians(thetaInitial);
		thetaFinal = Math.toRadians(thetaFinal);

		double x2 = lInitial * Math.cos(thetaInitial) + xInitial;
		double x3 = lFinal * Math.cos(thetaFinal + Math.PI) + xFinal;
		double y2 = lInitial * Math.sin(thetaInitial) + yInitial;
		double y3 = lFinal * Math.sin(thetaFinal + Math.PI) + yFinal;

		System.out.println("Forward Spline Generating");

		drivingController.addSpline(xInitial, x2, x3, xFinal, yInitial, y2, y3, yFinal,
				maxAcceleration, maxVelocity, startVelocity, endVelocity, true);
	}

	public void addBackwardsSpline(double xInitial, double yInitial, double thetaInitial, double lInitial,
			double xFinal, double yFinal, double thetaFinal, double lFinal, double maxAcceleration,
			double maxVelocity, double startVelocity, double endVelocity) {

		thetaInitial = Math.toRadians(thetaInitial);
		thetaFinal = Math.toRadians(thetaFinal);

		double x2 = lInitial * Math.cos(thetaInitial + Math.PI) + xInitial;
		double x3 = lFinal * Math.cos(thetaFinal) + xFinal;
		double y2 = lInitial * Math.sin(thetaInitial + Math.PI) + yInitial;
		double y3 = lFinal * Math.sin(thetaFinal) + yFinal;

		System.out.println("Backwards Spline Generating");

		drivingController.addSpline(xInitial, x2, x3, xFinal, yInitial, y2, y3, yFinal,
				maxAcceleration, maxVelocity, startVelocity, endVelocity, false);
	}

	@Override
	public void registerCommands() {
		new SubsystemCommand(this.registeredCommands, "driver_control") {

			@Override
			public void initialize() {
				driverControlled = true;
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
				// System.out.println("Right Encoder: " + rightShaftEncoder.getDistance() + "\tLeft Encoder: " + leftShaftEncoder.getDistance());
				// System.out.println("X = " + odometer.getCurrentX() + "|| Y = " + odometer.getCurrentY());

				// System.out.println("Odometer heading angle " + odometer.getHeadingAngle());
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

		new SubsystemCommand(this.registeredCommands, "brake_mode") {

			@Override
			public void initialize() {
				lMotor0.setIdleMode(CANSparkMax.IdleMode.kBrake);
				lMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
				lMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
				rMotor0.setIdleMode(CANSparkMax.IdleMode.kBrake);
				rMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
				rMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
			}

			@Override
			public void execute() {}

			@Override
			public boolean isFinished() {
				return false;
			}

			@Override
			public void end() {
				lMotor0.setIdleMode(CANSparkMax.IdleMode.kBrake);
				lMotor1.setIdleMode(CANSparkMax.IdleMode.kCoast);
				lMotor2.setIdleMode(CANSparkMax.IdleMode.kCoast);
				rMotor0.setIdleMode(CANSparkMax.IdleMode.kBrake);
				rMotor1.setIdleMode(CANSparkMax.IdleMode.kCoast);
				rMotor2.setIdleMode(CANSparkMax.IdleMode.kCoast);
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
			public void execute() {}

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
			public void execute() {}

			@Override
			public boolean isFinished() {
				return true;
			}

			@Override
			public void end() {}
		};

		new SubsystemCommand(this.registeredCommands, "debug_print") {

			@Override
			public void initialize() {

				lMotor0.setIdleMode(CANSparkMax.IdleMode.kCoast);
				rMotor0.setIdleMode(CANSparkMax.IdleMode.kCoast);

				lMotor0.set(0);
				rMotor0.set(0);

				System.out.println(odometer.getCurrentX() + " : " + odometer.getCurrentY());
			}

			@Override
			public void execute() {
				getEncoderValues();
				System.out.println("Heading Angle: " + odometer.getHeadingAngle());
				System.out.println("X : Y = " + odometer.getCurrentX() + " : " + odometer.getCurrentY());
			}

			@Override
			public boolean isFinished() {
				return false;
			}

			@Override
			public void end() {}
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
			public void execute() {}

			@Override
			public boolean isFinished() {
				return true;
			}

			@Override
			public void end() {}
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
			public void execute() {}

			@Override
			public boolean isFinished() {
				return true;
			}

			@Override
			public void end() {}
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
			public void execute() {}

			@Override
			public boolean isFinished() {
				return true;
			}

			@Override
			public void end() {}
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
			public void execute() {}

			@Override
			public boolean isFinished() {
				return true;
			}

			@Override
			public void end() {}
		};

		new SubsystemCommand(this.registeredCommands, "add_backwards_line") {

			@Override
			public void initialize() {
				System.out.println("Position Starting backwards line = " + "(" + odometer.getCurrentX()
						+ ", " + odometer.getCurrentY() + ")");

				double xInitial = Double.parseDouble(this.args[0]);
				double yInitial = Double.parseDouble(this.args[1]);
				double xFinal = Double.parseDouble(this.args[2]);
				double yFinal = Double.parseDouble(this.args[3]);

				drivingController.addSpline(xInitial, xInitial, xFinal, xFinal, yInitial, yInitial, yFinal, yFinal,
						Double.parseDouble(this.args[4]), Double.parseDouble(this.args[5]),
						Double.parseDouble(this.args[6]), Double.parseDouble(this.args[7]), false);

			}

			@Override
			public void execute() { }

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
				drivingController.setIsFinished(false);
				enable();
				System.out.println("starting path");
				// System.out.println(drivingController.getControlPath());
			}

			@Override
			public void execute() {

			 }

			@Override
			public boolean isFinished() {
				// System.out.println("driving controller finished? " + ":" + drivingController.isFinished());
				return drivingController.isFinished();
			}

			@Override
			public void end() {
				disable();
				closedLoopArcade(0, 0);
				System.out.println("x : y " + odometer.getCurrentX() + " : " + odometer.getCurrentY() + "Final Heading : " + odometer.getHeadingAngle());
			}
		};

		new SubsystemCommand(this.registeredCommands, "start_endless_path") {

			@Override
			public void initialize() {
				drivingController.setIsFinished(false);
				enable();
				System.out.println("starting path");
				// System.out.println(drivingController.getControlPath());
			}

			@Override
			public void execute() {

			}

			@Override
			public boolean isFinished() {
				// System.out.println("driving controller finished? " + ":" + drivingController.isFinished());
				return drivingController.isFinished();
			}

			@Override
			public void end() {
				disable();
				System.out.println("x : y" + odometer.getCurrentX() + " : " + odometer.getCurrentY() + "Final Heading : " + odometer.getHeadingAngle());
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

				// if (tx > 0){
				// 	System.out.println("Turning Right Pivot: " + pivot);
				// 	closedLoopTank((power * maxVelocity) + pivot, (power * maxVelocity));
				// } else if (tx < 0){
				// 	System.out.println("Turning Left Pivot: " + pivot);
				// 	closedLoopTank((power * maxVelocity), (power * maxVelocity) - pivot);
				// }


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

		new SubsystemCommand(this.registeredCommands, "turn_to_angle"){
			double requestedDelta;
			double finalRequestedAngle;

			PID headingController = new PID(0.01, 0.0001, 0, 0);

			@Override
			public void initialize() {
				try{
					requestedDelta = Double.parseDouble(this.args[0]);
				} catch(Exception foo) {
					System.out.println("Oof, forgot to enter an argument?");
				}
				finalRequestedAngle = odometer.getHeadingAngle() + requestedDelta;
				System.out.println("NavX Turn to Angle Command Aim:- " + finalRequestedAngle);
				headingController.setOutputLimits(-0.6, 0.6);
				headingController.setSetpoint(finalRequestedAngle);
			}

			@Override
			public void execute() {
				double errorCorrection = headingController.getOutput(odometer.getHeadingAngle());
				lMotor0.set(-errorCorrection);
				rMotor0.set(-errorCorrection);
			}

			@Override
			public boolean isFinished() {
				return false;
			}

			@Override
			public void end() {
				closedLoopArcade(0,0);
				
				System.out.println("Finished turn to angle, expected angle was " + finalRequestedAngle +
						" and your actual angle was " + odometer.getHeadingAngle() +
						". Error of " + (Math.abs(odometer.getHeadingAngle() - finalRequestedAngle)));
				System.out.println("Final turn to angle X: " + odometer.getCurrentX() + " Y: " + odometer.getCurrentY());
			}
		};

		new SubsystemCommand(this.registeredCommands, "turn_to_angle_setpoint"){
			double finalRequestedAngle;
			double startTime;

			PID headingController = new PID(0.01, 0.0001, 0, 0);

			@Override
			public void initialize() {
				finalRequestedAngle = Double.parseDouble(this.args[0]);
				startTime = System.nanoTime();

				System.out.println("Start turn_to_angle\tStart: " + odometer.getHeadingAngle() + "\tEnd: " + finalRequestedAngle);
				headingController.setOutputLimits(-0.2, 0.2);
				headingController.setSetpoint(finalRequestedAngle);
			}

			@Override
			public void execute() {
				double errorCorrection = headingController.getOutput(odometer.getHeadingAngle());
				lMotor0.set(-errorCorrection);
				rMotor0.set(-errorCorrection);
			}

			@Override
			public boolean isFinished() {
				return Math.abs(odometer.getHeadingAngle() - finalRequestedAngle) < 4 || System.nanoTime() - startTime > 3e9;
			}

			@Override
			public void end() {
				closedLoopArcade(0,0);

				System.out.println("Finished turn to angle, expected angle was " + finalRequestedAngle +
						" and your actual angle was " + odometer.getHeadingAngle() +
						". Error of " + (Math.abs(odometer.getHeadingAngle() - finalRequestedAngle)));
			}
		};

		new SubsystemCommand(this.registeredCommands, "auton_vision_align"){
			double counter;
			boolean isAboveMax = false;
			double maxBlobArea = 6;
			double currentBlobArea;
			double startingTime;

			@Override
			public void initialize() {
				counter = 0;
				System.out.println("INITIALIZED VISION ALIGN");
				limelightTable.getEntry("ledMode").setNumber(3);
				limelightTable.getEntry("camMode").setNumber(0);
				isAboveMax = false;
				startingTime = System.nanoTime();
			}


			@Override
			public void execute() {
				// System.out.println("Running");
				double tx = limelightTable.getEntry("tx").getDouble(0);
				currentBlobArea = limelightTable.getEntry("ta").getDouble(0);

				double kAngleP = 0.05;
				double kDistanceDivisor = 0.3; // Untested value. Direct proportionality.

				if (this.args[0] != null)
					maxBlobArea = Double.parseDouble(this.args[0]);

				double power = 0;
				if (currentBlobArea < maxBlobArea && currentBlobArea != 0)
					power = kDistanceDivisor / currentBlobArea;
				else
					power = 0;

				double pivot = tx * kAngleP;

				// System.out.println("kDistanceDivisor: " + kDistanceDivisor + "| blobArea : " + currentBlobArea);

				if (power > 0.2)
					power = 0.2;

				// System.out.println("power: " + power);

				if (currentBlobArea <= maxBlobArea) {
					counter = 0;
					closedLoopArcade(power * maxVelocity, -pivot);
				} else {
					counter++;
				}

				if (counter > 10) {
					isAboveMax = currentBlobArea > maxBlobArea;
				}
//				System.out.println("RUNNING VISION ALIGN POWER : " + power + " IS ABOVE MAX? : " + isAboveMax);


			}

			@Override
			public boolean isFinished() {

				// System.out.println("Boolean : " + (currentBlobArea > maxBlobArea));
					return ((System.nanoTime() - startingTime) > 2e9) || isAboveMax;
			}

			@Override
			public void end() {
				// limelightTable.getEntry("camMode").setNumber(1);
				closedLoopArcade(0, 0);
				lMotor0.set(0.0);
				rMotor0.set(0.0);
				// limelightTable.getEntry("ledMode").setNumber(1);
				System.out.println("VISION ALIGN FINAL POSITIONS x: " + odometer.getCurrentX() + " y: " + odometer.getCurrentY() + " thetaF: " + odometer.getHeadingAngle() + " COUNTER = " + counter);
			}
		};

		new SubsystemCommand(this.registeredCommands, "target_align"){
			double tx;

			@Override
			public void initialize() {

			}

			@Override
			public void execute() {
				double tx = limelightTable.getEntry("tx").getDouble(0);
				double kP = 0.05;
				double pivot = tx * kP;
				closedLoopArcade(0,-pivot);
			}

			@Override
			public boolean isFinished() {
				return tx < 2;
			}

			@Override
			public void end() {
				closedLoopTank(0,0);
			}
		};


		new SubsystemCommand(this.registeredCommands, "set_current_position") {
			@Override
			public void initialize() {
				odometer.setCurrentPosition(Double.parseDouble(this.args[0]), Double.parseDouble(this.args[1]));
				// System.out.println("SET POSITIONS: " + " X = " + odometer.getCurrentX() + " Y = " + odometer.getCurrentY());	
			}

			@Override
			public boolean isFinished() {
				return true;
			}

			@Override
			public void end() {
				System.out.println("Set to X: " + odometer.getCurrentX() + " | Y: " + odometer.getCurrentY());
			}
		};

		new SubsystemCommand(this.registeredCommands, "cancel_all") {

			@Override
			public void initialize() {
				controlsProcessor.cancelAll();
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

	}

}