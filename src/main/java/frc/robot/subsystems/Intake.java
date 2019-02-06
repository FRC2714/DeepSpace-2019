package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.util.ControlsProcessor;
import frc.robot.util.SubsystemCommand;
import frc.robot.util.SubsystemModule;

public class Intake extends SubsystemModule {

	// Controls Processor
	private ControlsProcessor controlsProcessor;

	// Intake Motors
	private CANSparkMax cargoMotor = new CANSparkMax(9, MotorType.kBrushless);
	private CANSparkMax hatchplatePump = new CANSparkMax(10, MotorType.kBrushless);

	// Hatchplate Servo
	private Servo hatchplateServo0 = new Servo(0);
	private Servo hatchplateServo1 = new Servo(1);

	private final double hatchplateUp = 80;
	private final double hatchplateDown = 100;

    public Intake(ControlsProcessor controlsProcessor) { 
		registerCommands(); // Puts commands onto the hashmap
		
		this.controlsProcessor = controlsProcessor;
	}
	


    @Override public void run() {

    }

    @Override public void registerCommands() {

        new SubsystemCommand(this.registeredCommands, "intake") {

			@Override
			public void initialize() {
				cargoMotor.set(0.75);
			}

			@Override
			public void execute() {

			}

			@Override
			public boolean isFinished() {
				return false;
			}

			@Override
			public void end() {
				cargoMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
				cargoMotor.set(0.05);
			}
		};

        new SubsystemCommand(this.registeredCommands, "extake") {

			@Override
			public void initialize() {
				cargoMotor.set(-0.5);
			}

			@Override
			public void execute() {

			}

			@Override
			public boolean isFinished() {
				return false;
			}

			@Override
			public void end() {
				cargoMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
				cargoMotor.set(0);
			}
		};

        new SubsystemCommand(this.registeredCommands, "hatchplate_up") {

			@Override
			public void initialize() {
				hatchplateServo0.set(1);
				hatchplateServo1.set(1);
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

        new SubsystemCommand(this.registeredCommands, "hatchplate_down") {

			@Override
			public void initialize() {
				hatchplateServo0.set(0.47);
				hatchplateServo1.set(0.47);
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

        new SubsystemCommand(this.registeredCommands, "hatchplate_pump") {

			@Override
			public void initialize() {
				hatchplatePump.set(Double.parseDouble(args[0]));
			}

			@Override
			public void execute() {

			}

			@Override
			public boolean isFinished() {
				return false;
			}

			@Override
			public void end() {
				hatchplatePump.set(0);
			}
		};
    }

	@Override
	public void init() {
		cargoMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		hatchplatePump.setIdleMode(CANSparkMax.IdleMode.kBrake);

		cargoMotor.set(0);
		hatchplatePump.set(0);
	}

	@Override
	public void destruct() {
		cargoMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		hatchplatePump.setIdleMode(CANSparkMax.IdleMode.kBrake);

		cargoMotor.set(0);
		hatchplatePump.set(0);
	}
}