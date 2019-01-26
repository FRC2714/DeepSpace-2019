package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;
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
	public DriveTrain drivetrain = new DriveTrain();

	// Initialize auton mode selector
	private Command autonomousCommand;
	private SendableChooser<Command> autoChooser;

	// Initialize robot control systems
	public ControlsProcessor ControlsProcessor;

	// Init and Periodic functions
	@Override
	public void robotInit() {
		autoChooser = new SendableChooser<>();
		SmartDashboard.putData("Autonomous Mode Selector", autoChooser);

		ControlsProcessor = new ControlsProcessor(2000000, 1) {
			@Override
			public void registerOperatorControls() {
				append("closed_loop_tank -s 5", this.a);
				System.out.println("Controls reg");
				//append("add_forwards_spline -s 0,0,-6,-6,0,3,6,9,8,8,0,0", this.y);
				//append("start_path -s", this.b);
			}
		};
		ControlsProcessor.registerController("DriveTrain", drivetrain);
		ControlsProcessor.start();
	}

	@Override
	public void disabledInit() {
		drivetrain.destruct();
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
		System.out.println("IT WORKS");
		Scheduler.getInstance().run();


		if(Math.abs(ControlsProcessor.getLeftJoystick()) >= 0.07 || Math.abs(ControlsProcessor.getRightJoystick()) >= 0.07){
			drivetrain.closedLoopArcade(-ControlsProcessor.getLeftJoystick() * drivetrain.getMaxVelocity(),
					ControlsProcessor.getRightJoystick());
		} else {
			drivetrain.closedLoopArcade(0,0);
		}

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

	// General init 
	private void generalInit() {
		if (ControlsProcessor != null) {
			ControlsProcessor.enable();
		}

		drivetrain.init();
	}
}