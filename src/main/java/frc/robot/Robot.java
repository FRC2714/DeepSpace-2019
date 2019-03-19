package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autontasks.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
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
	private Climber climber;

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
				// Go to start position
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
				append("hatch_station_intake -s", this.launchpad.getButtonInstance(3, 7));
				append("floor_hatch_position -p", this.launchpad.getButtonInstance(3, 8));
				append("hatch_station_intake -s", this.launchpad.getButtonInstance(3, 8));
				append("floor_hatch_position -p", this.launchpad.getButtonInstance(4, 7));
				append("hatch_station_intake -s", this.launchpad.getButtonInstance(4, 7));
				append("floor_hatch_position -p", this.launchpad.getButtonInstance(4, 8));
				append("hatch_station_intake -s", this.launchpad.getButtonInstance(4, 8));

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
				append("valve_off -s", this.launchpad.getButtonInstance(3, 4));
				append("station_position -p", this.launchpad.getButtonInstance(3, 5));
				append("hatch_station_intake -s", this.launchpad.getButtonInstance(3, 5));
				append("valve_off -s", this.launchpad.getButtonInstance(3, 5));
				append("station_position -p", this.launchpad.getButtonInstance(4, 4));
				append("hatch_station_intake -s", this.launchpad.getButtonInstance(4, 4));
				append("valve_off -s", this.launchpad.getButtonInstance(4, 4));
				append("station_position -p", this.launchpad.getButtonInstance(4, 5));
				append("hatch_station_intake -s", this.launchpad.getButtonInstance(4, 5));
				append("valve_off -s", this.launchpad.getButtonInstance(4, 5));

				// Score positions
				append("lower_score -p", this.launchpad.getButtonInstance(6, 8));
				append("lower_score -p", this.launchpad.getButtonInstance(7, 8));
				append("middle_score -p", this.launchpad.getButtonInstance(6, 6));
				append("middle_score -p", this.launchpad.getButtonInstance(7, 6));
				append("upper_score -p", this.launchpad.getButtonInstance(6, 4));
				append("upper_score -p", this.launchpad.getButtonInstance(7, 4));
				append("cargo_station_score -p", this.launchpad.getButtonInstance(6, 2));
				append("cargo_station_score -p", this.launchpad.getButtonInstance(7, 2));

				// Extake buttons
				// append("extake -s", this.launchpad.getButtonInstance(0, 1));
				append("extake -s", this.launchpad.getButtonInstance(0, 2));
				// append("extake -s", this.launchpad.getButtonInstance(1, 1));
				append("extake -s", this.launchpad.getButtonInstance(1, 2));

				// Jog
				append("jog_up -s", this.launchpad.getButtonInstance(8, 6));
				append("jog_down -s", this.launchpad.getButtonInstance(8, 8));

				// Game Piece Override
				append("cargo_true -p", this.launchpad.getButtonInstance(8, 1));
				append("hatch_true -p", this.launchpad.getButtonInstance(8, 3));

				// Driver controls
				append("driver_control -p", this.rightStick);
				append("vision_align -s", this.lb);
				append("break_mode -s", this.rb);

				// Climber
				append("climber_up -s", this.launchpad.getButtonInstance(0, 0));
				append("climber_pump -p", this.launchpad.getButtonInstance(0, 0));
				append("climber_down -s", this.launchpad.getButtonInstance(1, 0));

				// Oh no! Plz stop Brisket
				append("cancel_all -p", this.launchpad.getButtonInstance(7, 0));
				append("cancel_all -p", this.launchpad.getButtonInstance(8, 0));
			}
		};

		drivetrain = new DriveTrain(controlsProcessor);
		arm = new Arm(controlsProcessor);
		climber = new Climber();

		// Required to register all subsystems in order to be processed. 
		controlsProcessor.registerController("DriveTrain", drivetrain);
		controlsProcessor.registerController("Arm", arm);
		controlsProcessor.registerController("Climber", climber);

		controlsProcessor.start();
		
		arm.init();
	}

	/**
	 * Runs when the robot is disabled and cancels
	 * everything running in the controls processor
	 */
	@Override
	public void disabledInit() {
		drivetrain.destruct();
		arm.destruct();
		climber.destruct();
		Scheduler.getInstance().removeAll();

		if (controlsProcessor != null) {
			System.out.println("Disabled, clearing and disabling controlsProcessor");
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
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);

		generalInit();
		
		AutonTask leftFullSend = new LeftRocketHabTwoAuton(controlsProcessor);
		AutonTask leftCargo = new LeftCargoHabTwoAuton(controlsProcessor);

		AutonTask rightFullSend = new RightRocketHabTwoAuton(controlsProcessor);
		AutonTask rightCargo = new RightCargoHabTwoAuton(controlsProcessor);

		leftCargo.run();
	}

	/**
	 * Runs periodically during auton
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * Runs at the start of teleop mode
	 */
	@Override
	public void teleopInit() {
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
		if (autonomousCommand != null)
			autonomousCommand.cancel();

		generalInit();
		drivetrain.closedLoopArcade(0,0);
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
	public void testInit(){
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
	}

	/**
	 * Unused
	 */
	@Override
	public void testPeriodic() {
	}

	/**
	 * Called at the start of both auton and teleop init
	 */
	private void generalInit() {
		controlsProcessor.cancelAll();

		if (controlsProcessor != null) {
			controlsProcessor.enable();
		}

		drivetrain.init();
		climber.init();
	}
}