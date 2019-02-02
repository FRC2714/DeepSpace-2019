package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autontasks.TestAuton;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

/*
  The VM is configured to automatically run this class, and to call the
  functions corresponding to each mode, as described in the IterativeRobot
  documentation. If you change the name of this class or the package after
  creating this project, you must also update the manifest file in the resource
  directory.
*/
public class Robot extends TimedRobot {

	// Initialize subsystems
	private DriveTrain drivetrain;
	private Arm arm;

	// Initialize auton mode selector
	private Command autonomousCommand;
	private SendableChooser<Command> autoChooser;

	// Initialize robot control systems
	private ControlsProcessor controlsProcessor;

	// Init and Periodic functions
	@Override
	public void robotInit() {
		autoChooser = new SendableChooser<>();
		SmartDashboard.putData("Autonomous Mode Selector", autoChooser);

		// Controls processor only gets created ONCE when code is run
		controlsProcessor = new ControlsProcessor(2000000, 10) {
			@Override
			public void registerOperatorControls() {
				// append("jog_up -s", this.y);
				// append("jog_down -s", this.a);
				append("go_to_position_basic -p 90,180,45,90", this.b);
				append("go_to_position_basic -p 155,180,90,90", this.a);
				append("go_to_position_basic -p 10,180,0,90", this.x);
			}
		};

		drivetrain = new DriveTrain(controlsProcessor);
		arm = new Arm(controlsProcessor);

		// Required to register all subsystems in order to be processed. 
		controlsProcessor.registerController("DriveTrain", drivetrain);
		controlsProcessor.registerController("Arm", arm);
		controlsProcessor.start();
	}

	/**
	 * Runs when the robot is disabled and cancels
	 * everything running in the controls processor
	 */
	@Override
	public void disabledInit() {
		drivetrain.destruct();
		arm.destruct();
		Scheduler.getInstance().removeAll();

		if (controlsProcessor != null) {
			controlsProcessor.cancelAll();
			controlsProcessor.disable();
		}
	}

	/**
	 * Does NOTHING!
	 */
	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * Runs at the beginning of auton mode
	 * TODO: Replace WPI Lib with auton task
	 */
	@Override
	public void autonomousInit() {
		generalInit();

		AutonTask choiceOne = new TestAuton(controlsProcessor);
		AutonTask choiceTwo = new TestAuton(controlsProcessor);

		choiceOne.run();
	}

	/**
	 * Runs periodically during auton
	 */
	@Override
	public void autonomousPeriodic() { Scheduler.getInstance().run(); }

	/**
	 * Runs at the start of teleop mode
	 * TODO: Cancel the queue
	 */
	@Override
	public void teleopInit() {
		if (autonomousCommand != null)
			autonomousCommand.cancel();

		generalInit();
	}

	/**
	 * Runs periodically during teleop
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * Unused
	 */
	@Override
	public void testInit() { }

	/**
	 * Unused
	 */
	@Override
	public void testPeriodic() { }

	/**
	 * Called at the start of both auton and teleop init
	 */
	private void generalInit() {
		if (controlsProcessor != null) {
			controlsProcessor.enable();
		}

		drivetrain.init();
		arm.init();
	}
}