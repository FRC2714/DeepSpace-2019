package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.ControllerCollection;

/*
  The VM is configured to automatically run this class, and to call the
  functions corresponding to each mode, as described in the IterativeRobot
  documentation. If you change the name of this class or the package after
  creating this project, you must also update the manifest file in the resource
  directory.
*/
public class Robot extends TimedRobot {

	// Initialize subsystems
	public DriveTrain drivetrain = new DriveTrain();

	// Initialize auton mode selector
	private Command autonomousCommand;
	private SendableChooser<Command> autoChooser;

	// Initialize robot control systems
	public ControllerCollection ControlsProcessor;
	public OI ControlInterface;

	// Init and Periodic functions
	@Override
	public void robotInit() {

		autoChooser = new SendableChooser<>();

		SmartDashboard.putData("Autonomous Mode Selector", autoChooser);
	}

	@Override
	public void disabledInit() {
		drivetrain.drivetrainDestruct();
		Scheduler.getInstance().removeAll();

		if (ControlsProcessor != null) {
			ControlsProcessor.cancelAll();
			ControlsProcessor.disable();
		}
		
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

	@Override
	public void testPeriodic() {

	}

	// Init function that is used for all types of init(Auton, Teleop, etc.)
	private void generalInit() {
		if (ControlsProcessor != null) {
			ControlsProcessor.enable();
		} else {
			ControlsProcessor = new ControllerCollection(500000, 1);
			ControlsProcessor.registerController("DriveTrain", drivetrain);
			ControlsProcessor.start();

			ControlInterface = new OI(ControlsProcessor);
		}

		drivetrain.drivetrainInit();
	}
}