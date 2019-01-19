package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;
// import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Intake;
import frc.robot.util.ControllerCollection;

/
  The VM is configured to automatically run this class, and to call the
  functions corresponding to each mode, as described in the IterativeRobot
  documentation. If you change the name of this class or the package after
  creating this project, you must also update the manifest file in the resource
  directory.
 /
public class Robot extends TimedRobot {

	//Initialize 
	public DriveTrain drivetrain = new DriveTrain();

	private static Command autonomousCommand;
	private SendableChooser<Command> autoChooser;

	public static ControllerCollection ControlsProcessor = null;
	public static double splinePercentage;

	/
	  This function is run when the robot is first started up and should be used
	  for any initialization code.
	 /
	@Override
	public void robotInit() {

		autoChooser = new SendableChooser<>();

		SmartDashboard.putData("Autonomous Mode Selector", autoChooser);
	}

	/
	  This function is called once each time the robot enters Disabled mode. You
	  can use it to reset any subsystem information you want to clear when the
	  robot is disabled.
	 /
	@Override
	public void disabledInit() {
		System.out.println("Disabled init start");
		drivetrain.drivetrainDestruct();
		Scheduler.getInstance().removeAll();

		if (ControlsProcessor != null) {
			System.out.println("Not Null");
			ControlsProcessor.cancelAll();
			ControlsProcessor.stopProcessor();
		}
		System.out.println("Disabled init end");
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {

		autonomousCommand = autoChooser.getSelected();

		if (autonomousCommand != null) {
			autonomousCommand.start();
		}

		generalInit();

	}

	/
	  This function is called periodically during autonomous
	 /
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

	/
	  This function is called periodically during operator control
	 /
	@Override
	public void teleopPeriodic() {

		Scheduler.getInstance().run();

	}

	@Override
	public void testInit() {

		autonomousCommand = autoChooser.getSelected();

		if (autonomousCommand != null) {
			autonomousCommand.start();
		}

		generalInit();

	}

	/
	  This function is called periodically during autonomous
	 /
	@Override
	public void testPeriodic() {

	}

	// Init function that is used for all types of init(Auton, Teleop, etc.)
	private void generalInit() {
		if (ControlsProcessor != null) {
			ControlsProcessor.stopProcessor = false;
		} else {
			ControlsProcessor = new ControllerCollection(500000, 1);
			ControlsProcessor.registerController("DriveTrain", Robot.drivetrain);
			ControlsProcessor.start();
		}

		drivetrain.drivetrainInit();
	}
}