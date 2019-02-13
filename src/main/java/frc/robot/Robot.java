package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import frc.robot.autontasks.DelayAutonTesterTask;
import frc.robot.autontasks.LeftCargoHatchAuton;
import frc.robot.autontasks.LeftRocketHatchAuton;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
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
	private Intake intake;

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
		controlsProcessor = new ControlsProcessor(10000000, 2) {
			@Override
			public void registerOperatorControls() {
				append("closed_loop_tank -s 2", this.launchpad.getButtonInstance(0, 0));
				append("closed_loop_tank -s -2", this.launchpad.getButtonInstance(1, 0));
				
				// append("go_to_position -p 53,70,195,70", this.a); // Lower cargo rocket
				append("go_to_position -p 85,70,200,200", this.b); // Middle cargo rocket
				// append("go_to_position -p 110,70,230.8,70", this.y); // Top cargo rocket (No good)
				
				// append("go_to_position -p 15,70,80,70", this.x); // Lower hatch rocket
				// append("go_to_position -p 62,70,122,70", this.y); // Middle hatch rocket
				// append("go_to_position -p 110,70,230.8,70", this.y); // Top hatch rocket (No good)

				// append("go_to_position -p 22.75,70,182,70", this.x); // Intake floor

				//append("hatchplate_up -p", this.a);
				// append("hatchplate_down -p", this.b);

				// append("intake -s", this.rb);
				// append("extake -s", this.lb);

				append("driver_control -p", this.rightStick);
				append("debug_print -s", this.a);
			}
		};

		drivetrain = new DriveTrain(controlsProcessor);
		arm = new Arm(controlsProcessor);
		intake = new Intake(controlsProcessor);

		// Required to register all subsystems in order to be processed. 
		controlsProcessor.registerController("DriveTrain", drivetrain);
		controlsProcessor.registerController("Arm", arm);
		controlsProcessor.registerController("Intake", intake);
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
		intake.destruct();
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
	 */
	@Override
	public void autonomousInit() {
		generalInit();
		
		AutonTask leftRocket = new LeftRocketHatchAuton(controlsProcessor);
		AutonTask leftCargo = new LeftCargoHatchAuton(controlsProcessor);

		leftCargo.run();
	}

	/**
	 * Runs periodically during auton
	 */
	@Override
	public void autonomousPeriodic() { Scheduler.getInstance().run(); }

	/**
	 * Runs at the start of teleop mode
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
		//drivetrain.odometer.printEncoderPosition();
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
		intake.init();
	}
}