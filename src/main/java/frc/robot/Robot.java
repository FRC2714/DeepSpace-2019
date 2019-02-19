package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autontasks.*;
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
		controlsProcessor = new ControlsProcessor(10000000, 2) {
			@Override
			public void registerOperatorControls() {
				// Go to start postion
				append("start_position -p", this.launchpad.getButtonInstance(3, 1));
				append("start_position -p", this.launchpad.getButtonInstance(3, 2));
				append("start_position -p", this.launchpad.getButtonInstance(4, 1));
				append("start_position -p", this.launchpad.getButtonInstance(4, 2));

				// Intake cargo from ground
				append("floor_cargo_position -p", this.launchpad.getButtonInstance(0, 7));
				append("cargo_intake -s", this.launchpad.getButtonInstance(0, 7));
				append("floor_cargo_position -p", this.launchpad.getButtonInstance(0, 8));
				append("cargo_intake -s", this.launchpad.getButtonInstance(0, 8));
				append("floor_cargo_position -p", this.launchpad.getButtonInstance(1, 7));
				append("cargo_intake -s", this.launchpad.getButtonInstance(1, 7));
				append("floor_cargo_position -p", this.launchpad.getButtonInstance(1, 8));
				append("cargo_intake -s", this.launchpad.getButtonInstance(1, 8));

				// Intake hatch from ground
				append("floor_hatch_position -p", this.launchpad.getButtonInstance(3, 7));
				append("hatch_floor_intake -s", this.launchpad.getButtonInstance(3, 7));
				append("floor_hatch_position -p", this.launchpad.getButtonInstance(3, 8));
				append("hatch_floor_intake -s", this.launchpad.getButtonInstance(3, 8));
				append("floor_hatch_position -p", this.launchpad.getButtonInstance(4, 7));
				append("hatch_floor_intake -s", this.launchpad.getButtonInstance(4, 7));
				append("floor_hatch_position -p", this.launchpad.getButtonInstance(4, 8));
				append("hatch_floor_intake -s", this.launchpad.getButtonInstance(4, 8));

				// Intake cargo from station
				append("station_position -p", this.launchpad.getButtonInstance(0, 4));
				append("cargo_intake -s", this.launchpad.getButtonInstance(0, 4));
				append("station_position -p", this.launchpad.getButtonInstance(0, 5));
				append("cargo_intake -s", this.launchpad.getButtonInstance(0, 5));
				append("station_position -p", this.launchpad.getButtonInstance(1, 4));
				append("cargo_intake -s", this.launchpad.getButtonInstance(1, 4));
				append("station_position -p", this.launchpad.getButtonInstance(1, 5));
				append("cargo_intake -s", this.launchpad.getButtonInstance(1, 5));

				// Intake hatch from station
				append("station_position -p", this.launchpad.getButtonInstance(3, 4));
				append("hatch_station_intake -s", this.launchpad.getButtonInstance(3, 4));
				append("station_position -p", this.launchpad.getButtonInstance(3, 5));
				append("hatch_station_intake -s", this.launchpad.getButtonInstance(3, 5));
				append("station_position -p", this.launchpad.getButtonInstance(4, 4));
				append("hatch_station_intake -s", this.launchpad.getButtonInstance(4, 4));
				append("station_position -p", this.launchpad.getButtonInstance(4, 5));
				append("hatch_station_intake -s", this.launchpad.getButtonInstance(4, 5));

				// Score positions
				append("lower_score -p", this.launchpad.getButtonInstance(6, 8));
				append("lower_score -p", this.launchpad.getButtonInstance(7, 8));
				append("middle_score -p", this.launchpad.getButtonInstance(6, 6));
				append("middle_score -p", this.launchpad.getButtonInstance(7, 6));
				append("upper_score -p", this.launchpad.getButtonInstance(6, 4));
				append("upper_score -p", this.launchpad.getButtonInstance(7, 4));
				append("back_score -p", this.launchpad.getButtonInstance(6, 2));
				append("back_score -p", this.launchpad.getButtonInstance(7, 2));

				// Extake
				append("extake -s", this.launchpad.getButtonInstance(0, 1));
				append("extake -s", this.launchpad.getButtonInstance(0, 2));
				append("extake -s", this.launchpad.getButtonInstance(1, 1));
				append("extake -s", this.launchpad.getButtonInstance(1, 2));

				// Jog
				append("jog_up -s", this.launchpad.getButtonInstance(8, 6));
				append("jog_down -s", this.launchpad.getButtonInstance(8, 8));

				// Game Piece Override
				append("cargo_true -p", this.launchpad.getButtonInstance(8, 1));
				append("hatch_true -p", this.launchpad.getButtonInstance(8, 3));

				// Toggle driver control
				append("driver_control -p", this.rightStick);
				append("get_arm_position -s", this.rb);

				// append("go_to_position -p 126,58", this.a);
				// append("debug_print -p", this.leftStick);

				append("intake_stop -s", this.launchpad.getButtonInstance(0, 0));
				// append("servo2 -p 0", this.b);
				// append("servo1 -p 0", this.x);
				// append("servo2 -p 0", this.y);

				append("drive_to_target -s 3", this.leftStick);
				append("debug_print -p", this.rb);

				// // Toggle end game
				// append("endgame_toggle -p", this.launchpad.getButtonInstance(0, 4))
				// append("endgame_toggle -p", this.launchpad.getButtonInstance(0, 5))
				// append("endgame_toggle -p", this.launchpad.getButtonInstance(0, 6))
				// append("endgame_toggle -p", this.launchpad.getButtonInstance(0, 7))
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
		if (controlsProcessor != null) {
			controlsProcessor.cancelAll();
			controlsProcessor.disable();
		}

		drivetrain.destruct();
		arm.destruct();
		Scheduler.getInstance().removeAll();
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
		AutonTask rightRocket = new RightRocketHatchAuton(controlsProcessor);
		AutonTask rightCargo = new RightCargoHatchAuton(controlsProcessor);
		AutonTask pickupHatch = new PickupAutonHatch(controlsProcessor);

		pickupHatch.run();
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
	}
}