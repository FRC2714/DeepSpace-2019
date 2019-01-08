package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
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

	// Motors for DriveTrain
	public Spark leftDrive1 = new Spark(RobotMap.p_leftDrive1);
	public Spark leftDrive2 = new Spark(RobotMap.p_leftDrive2);
	public Spark rightDrive1 = new Spark(RobotMap.p_rightDrive1);
	public Spark rightDrive2 = new Spark(RobotMap.p_rightDrive2);

	// Left and Right sides for DriveTrain
	public SpeedControllerGroup left = new SpeedControllerGroup(leftDrive1, leftDrive2);
	public SpeedControllerGroup right = new SpeedControllerGroup(rightDrive1, rightDrive2);

	// The DriveTrain drive
	public DifferentialDrive drive1 = new DifferentialDrive(left, right);

	// Encoders for the motors
	public Encoder leftEncoder = new Encoder(RobotMap.p_leftEncoderA, RobotMap.p_leftEncoderB, true, EncodingType.k4X);
	public Encoder rightEncoder = new Encoder(RobotMap.p_rightEncoderA, RobotMap.p_rightEncoderB, true,
			EncodingType.k4X);

	// Pneumatic for the gearbox
	//public DoubleSolenoid driveShifter = new DoubleSolenoid(RobotMap.p_driveShifter1, RobotMap.p_driveShifter2);

	// The NavX gyro
	public AHRS navX = new AHRS(SPI.Port.kMXP);

	// Initialize your subsystem here
	public DriveTrain() {
		registerCommands();
	}

	// Instantiate odometer and link in encoders and navX
	public Odometer odometer = new Odometer(0, 0) {
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
			drive1.arcadeDrive(power, pivot, false);
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
		drive1.arcadeDrive(power, pivot);
	}

	// Sets drive train to 0
	public void drivetrainSetPowerZero() {
		left.set(0);
		right.set(0);
	}

	@Override
	public void registerCommands() {

		/*
		 * 
		 * // High Gear new SubsystemCommand(this.registeredCommands, "shift_high", 0) {
		 * int doneTransition = 0;
		 * 
		 * @Override public void initialize() {
		 * Robot.drivetrain.leftEncoder.setDistancePerPulse(-0.0011146); //0.00116
		 * Robot.drivetrain.rightEncoder.setDistancePerPulse(0.0011181); //.00115
		 * 
		 * //Robot.drivetrain.driveShifter.set(DoubleSolenoid.Value.kForward); }
		 * 
		 * @Override public void execute() { if(doneTransition <= 0) {
		 * if((Math.abs(Robot.drivetrain.leftEncoder.getRate() +
		 * Robot.drivetrain.rightEncoder.getRate())/2)>12){
		 * Robot.drivetrain.driveShifter.set(DoubleSolenoid.Value.kForward);
		 * doneTransition = 700; } if((Math.abs(Robot.drivetrain.leftEncoder.getRate() +
		 * Robot.drivetrain.rightEncoder.getRate())/2)<8){
		 * Robot.drivetrain.driveShifter.set(DoubleSolenoid.Value.kReverse);
		 * doneTransition = 700; } } else { doneTransition--; } }
		 * 
		 * @Override public void end() {
		 * Robot.drivetrain.driveShifter.set(DoubleSolenoid.Value.kReverse);
		 * Robot.drivetrain.leftEncoder.setDistancePerPulse(-0.000623); //0.00116
		 * Robot.drivetrain.rightEncoder.setDistancePerPulse(0.000610); //.00115 }
		 * 
		 * @Override public boolean isFinished() { return false; } };
		 * 
		 * // Low Gear new SubsystemCommand(this.registeredCommands, "shift_low", 0) {
		 * 
		 * @Override public void initialize() {
		 * Robot.drivetrain.driveShifter.set(DoubleSolenoid.Value.kReverse); } };
		 * 
		 */

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