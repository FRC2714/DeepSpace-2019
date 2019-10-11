package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
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

	private Auton_Side auton_side;
	private Auton_Mode auton_mode;

	// Initialize subsystems
	private DriveTrain drivetrain;
	private Arm arm;
	private Climber climber;

	// Initialize auton mode selector
	private Command autonomousCommand;

	// Initialize robot control systems
	private ControlsProcessor controlsProcessor;

	// Init and Periodic functions
	@Override
	public void robotInit() {
		auton_side = Auton_Side.LEFT;
		auton_mode = Auton_Mode.ROCKET;

		// Controls processor only gets created ONCE when code is run
		controlsProcessor = new ControlsProcessor(10000000, 2) {
			@Override
			public void registerOperatorControls() {
				// Go to start position
				append("start_position -p", this.launchpad.getButtonInstance(3, 1));
				append("start_position -p", this.launchpad.getButtonInstance(3, 2));
				append("start_position -p", this.launchpad.getButtonInstance(4, 1));
				append("start_position -p", this.launchpad.getButtonInstance(4, 2));
				append("start_position -p", this.startPositionButton);

				// Intake cargo from ground
				append("floor_cargo_position -p", this.launchpad.getButtonInstance(0, 7));
				append("cargo_intake -s", this.launchpad.getButtonInstance(0, 7));
				append("floor_cargo_position -p", this.launchpad.getButtonInstance(0, 8));
				append("cargo_intake -s", this.launchpad.getButtonInstance(0, 8));
				append("floor_cargo_position -p", this.launchpad.getButtonInstance(1, 7));
				append("cargo_intake -s", this.launchpad.getButtonInstance(1, 7));
				append("floor_cargo_position -p", this.launchpad.getButtonInstance(1, 8));
				append("cargo_intake -s", this.launchpad.getButtonInstance(1, 8));
				append("floor_cargo_position -p", this.cargoFloorButton);
				append("cargo_intake -s", this.cargoFloorButton);

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
				append("station_position -p", this.cargoStationPositionButton);
				append("cargo_intake -s", this.cargoStationPositionButton);

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
				append("station_position -p",  this.hatchStationPositionButton);
				append("hatch_station_intake -s", this.hatchStationPositionButton);
				append("valve_off -s", this.hatchStationPositionButton);


				// Score positions
				append("lower_score -p", this.launchpad.getButtonInstance(6, 8));
				append("lower_score -p", this.launchpad.getButtonInstance(7, 8));
				append("middle_score -p", this.launchpad.getButtonInstance(6, 6));
				append("middle_score -p", this.launchpad.getButtonInstance(7, 6));
				append("upper_score -p", this.launchpad.getButtonInstance(6, 4));
				append("upper_score -p", this.launchpad.getButtonInstance(7, 4));
				append("flex_score -p", this.launchpad.getButtonInstance(6, 2));
				append("flex_score -p", this.launchpad.getButtonInstance(7, 2));

				append("lower_score -p", this.lowerScoreButton);
				append("middle_score -p", this.middleScoreButton);
				append("upper_score -p", this.upperScoreButton);
				append("flex_score -p", this.flexScoreButton);

				// Extake buttons
				append("extake -s", this.launchpad.getButtonInstance(0, 2));
				append("extake -s", this.launchpad.getButtonInstance(1, 2));
				append("extake -s", this.extakeButton);

				// Jog
				append("wrist_jog_up -s", this.launchpad.getButtonInstance(8, 5));
				append("wrist_jog_down -s", this.launchpad.getButtonInstance(8, 6));
				append("shoulder_jog_up -s", this.launchpad.getButtonInstance(8, 7));
				append("shoulder_jog_down -s", this.launchpad.getButtonInstance(8, 8));
//				append("wrist_jog_up -s", this.wristJogUpButton);
//				append("wrist_jog_down -s", this.wristJogDownButton);
//				append("shoulder_jog_up -s", this.armJogUpButton);
//				append("shoulder_jog_down -s", this.armJogDownButton);

				// Game Piece Override
				append("cargo_true -p", this.launchpad.getButtonInstance(8, 1));
				append("hatch_true -p", this.launchpad.getButtonInstance(8, 3));
//				append("cargo_true -p", this.cargoTrueButton);

				// Driver controls
				append("driver_control -p", this.rightStick);
				append("vision_align -s", this.lb);
				append("break_mode -s", this.rb);

				// Climber
				append("climber_up -s", this.launchpad.getButtonInstance(0, 0));
				append("climber_pump -p", this.launchpad.getButtonInstance(0, 0));
				append("climber_down -s", this.launchpad.getButtonInstance(1, 0));

				append("climber_up -s", this.climberUpButton);
				append("climber_pump -p", this.climberUpButton);
				append("climber_down -s", this.climberDownButton);

				// Zero the arm
				append("zero_arm -p", this.launchpad.getButtonInstance(5, 0));

				// Oh no! Plz stop Brisket
				append("cancel_all -p", this.launchpad.getButtonInstance(7, 0));
				append("cancel_all -p", this.launchpad.getButtonInstance(8, 0));

				append("cancel_all -p", this.cancelAllButton);

				append("hatch_intake -p", this.a);
				append("get_climber_positions -p", this.x);
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

		drivetrain.drivingController.clearControlPath();

		switch (auton_side){
			case LEFT:
				switch (auton_mode){
					case CARGO:
						System.out.println("GENERATING LEFT CARGO SPLINE ");
						NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(2);
						drivetrain.addBackwardsSpline(0, 0, 270, 7, -6,19.9,0, 4,6,12,0,0);
						break;
					case ROCKET:
						System.out.println("GENERATING LEFT ROCKET SPLINE ");
						drivetrain.addBackwardsSpline(0,0,270,7,-4.5,24,270,5,6,12,0,0);
						break;
					case ROCKETLOW:
						drivetrain.addBackwardsSpline(0,0,270,7,-4.5,24,270,5,6,12,0,0);
						break;
					case TEST:
						System.out.println("GENERATING LEFT TEST SPLINE ");
						break;
				}
				break;
			case RIGHT:
				switch (auton_mode){
					case CARGO:
						NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
						drivetrain.addBackwardsSpline(0, 0, 270, 7, 6,19.9,180, 4,6,12,0,0);
						break;
					case ROCKET:
						System.out.println("GENERATING RIGHT ROCKET SPLINE");
						drivetrain.addBackwardsSpline(0,0,270,7,4.5,24,270,5,6,12,0,0);
						break;
					case ROCKETLOW:
						drivetrain.addBackwardsSpline(0,0,270,7,4.5,24,270,5,6,12,0,0);
						break;
					case TEST:
						System.out.println("GENERATING RIGHT TEST SPLINE ");
//						drivetrain.addBackwardsSpline(0,0,270,2,5,20,306.75338685111194,2,12,12,0,0);
						drivetrain.addBackwardsSpline(0,0,270,7,4.5,25,270,5,6,12,0,0);
//						drivetrain.addForwardSpline(0,0,90,7,5.2,9,70,2,5,6,0,4);
						break;
				}
				break;
		}


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
	 *
	 * Does NOTHING!
	 */
	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * Represents the side that the autonomous starts from.
	 */
	enum Auton_Side{
		LEFT,
		RIGHT
	}

	/**
	 * Represents the task focused on during autonomous.
	 */
	enum Auton_Mode{
		CARGO,
		ROCKET,
		ROCKETLOW,
		TEST
	}

	/**
	 * Runs at the beginning of auton mode
	 */
	@Override
	public void autonomousInit() {
		drivetrain.odometer.reset();

		NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);

		generalInit();
		
		AutonTask leftRocket = new LeftRocket(controlsProcessor);
		AutonTask rightRocket = new RightRocket(controlsProcessor);

		AutonTask leftCargo = new LeftCargo(controlsProcessor);
		AutonTask rightCargo = new RightCargo(controlsProcessor);

		AutonTask leftRocketLowNonExistential = new TestTask(controlsProcessor);
		AutonTask rightRocketLow = new RightRocketLow(controlsProcessor);

		AutonTask testAuton = new TestTask(controlsProcessor);

		switch (auton_side){
			case LEFT:
				startAuton(leftCargo, leftRocket, leftRocketLowNonExistential, testAuton);
				break;
			case RIGHT:
				startAuton(rightCargo, rightRocket, rightRocketLow, testAuton);
				break;
		}
	}

	private void startAuton(AutonTask cargoAuton, AutonTask rocketAuton, AutonTask rocketLowAuton, AutonTask testAuton) {
		switch (auton_mode){
			case CARGO:
				cargoAuton.run();
				break;
			case ROCKET:
				rocketAuton.run();
				break;
			case ROCKETLOW:
				rocketLowAuton.run();
				break;
			case TEST:
				System.out.println("TEST CASE RUNNING IN AUTON INIT");
				testAuton.run();
				break;
		}
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
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
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
//		drivetrain.getEncoderValues();
//		SmartDashboard.putNumber("Gyro", drivetrain.odometer.getHeadingAngle());
		SmartDashboard.putNumber("Y Value", drivetrain.odometer.getCurrentY());
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
		if (controlsProcessor != null) {
			controlsProcessor.enable();
		}
		
		drivetrain.init();
		climber.init();

		controlsProcessor.cancelAll();
	}
}