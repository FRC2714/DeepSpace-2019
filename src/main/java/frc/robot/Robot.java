package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;
// import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Intake;
import frc.robot.util.ControllerCollection;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	public static final DriveTrain drivetrain = new DriveTrain();
	// public static final Elevator elevator = new Elevator();
	// public static final Intake intake = new Intake();

	public static OI oi;

	public static Command autonomousCommand;
	SendableChooser<Command> autoChooser;
	
	public static ControllerCollection ControlsProcessor = null;
	public static double splinePercentage;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		oi = new OI();
		Robot.drivetrain.leftEncoder.reset();
		Robot.drivetrain.rightEncoder.reset();
		Robot.drivetrain.navX.reset();
		Robot.drivetrain.leftEncoder.setReverseDirection(true);

		autoChooser = new SendableChooser<Command>();
		
		SmartDashboard.putData("Autonomous Mode Selector", autoChooser);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {		
		System.out.println("Disabled init start");
		Robot.drivetrain.navX.reset();
		Robot.drivetrain.leftEncoder.reset();
		Robot.drivetrain.rightEncoder.reset();
		Robot.drivetrain.drivetrainSetPowerZero();
		Scheduler.getInstance().removeAll();
		
		if(ControlsProcessor != null) {
			System.out.println("Not Null");
			ControlsProcessor.cancelAll();
			ControlsProcessor.stopProcessor();
		}
		System.out.println("Disabed init end");
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		
		autonomousCommand = (Command) autoChooser.getSelected();

		if (autonomousCommand != null) {
			autonomousCommand.start();
		}
		
		generalInit();
		
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		
		Scheduler.getInstance().run();

	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null)
			autonomousCommand.cancel();
		
		generalInit();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		
		// Arcade Drive
		if (Math.abs(oi.getLeftJoystick()) > .05 || Math.abs(oi.getRightJoystick()) > .05) {
			drivetrain.arcadeDrive(-oi.getLeftJoystick(), oi.getRightJoystick());
		} else {
			drivetrain.arcadeDrive(0, 0);
		}

		//System.out.println("encL: " + Robot.drivetrain.leftEncoder.get() + " encR: " + Robot.drivetrain.rightEncoder.get());
		//System.out.println("gfh: " + Robot.drivetrain.navX.getFusedHeading());
		System.out.println("X: " + Robot.drivetrain.odometer.current_x+ " Y: " + Robot.drivetrain.odometer.current_y);
		
	}

	@Override
	public void testInit() {
		
		autonomousCommand = (Command) autoChooser.getSelected();

		if (autonomousCommand != null) {
			autonomousCommand.start();
		}
		
		generalInit();
		
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void testPeriodic() {
		

	}

	//Init function that is used for all types of init(Auton, Teleop, etc.)
	public void generalInit() {
		if(ControlsProcessor != null) {
			System.out.println("resuming");
			ControlsProcessor.stopProcessor = false;
		} else {
			System.out.println("GeneralInit");

			ControlsProcessor = new ControllerCollection(500000, 1);
			ControlsProcessor.registerController("DriveTrain", Robot.drivetrain);
			// ControlsProcessor.registerController("Elevator", Robot.elevator);
			// ControlsProcessor.registerController("Intake", Robot.intake);
			ControlsProcessor.start();
		}
		Robot.drivetrain.navX.zeroYaw();
		Robot.drivetrain.leftEncoder.reset();
		Robot.drivetrain.rightEncoder.reset();
		Robot.drivetrain.odometer.reset();

		//Robot.drivetrain.driveShifter.set(DoubleSolenoid.Value.kReverse);
		Robot.drivetrain.leftEncoder.setDistancePerPulse(0.0336); //0.00116
		Robot.drivetrain.rightEncoder.setDistancePerPulse(-0.00105);
	}
}