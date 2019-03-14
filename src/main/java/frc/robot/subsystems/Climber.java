package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.util.SubsystemCommand;
import frc.robot.util.SubsystemModule;

public class Climber extends SubsystemModule {

    private CANSparkMax climberMotor = new CANSparkMax(11, MotorType.kBrushless);
    // private CANSparkMax climberPump = new CANSparkMax(12, MotorType.kBrushed);

    // MAX encoders
	private CANEncoder climberEncoder = climberMotor.getEncoder();

    // Climber positions
    private double climberPosition;

    // Double press for climb
    public long timeAtPress;
    public boolean climbMode;

    public Climber() { 
        registerCommands(); // Puts commands onto the hashmaps
    }

    /**
     * @return the position of the lifter in 
     */
    public void updatePositions() {
        climberPosition = climberEncoder.getPosition();
    }

    @Override public void run() {
        updatePositions();
    }

    @Override public void registerCommands() {

        new SubsystemCommand(this.registeredCommands, "climber_up") {

            @Override
			public void initialize() {
                if(System.nanoTime() - timeAtPress < 1000000000 || climbMode == true) {
                    climberMotor.set(1.0);
                    climbMode = true;
                } else {
                    timeAtPress = System.nanoTime();
                }
			}

			@Override
			public void execute() {
                if(climberPosition >= 500) // in rotations
                    climberMotor.set(0);
            }

			@Override
			public boolean isFinished() {
				return false;
			}

			@Override
			public void end() {
                System.out.println("Lifter: " + climberEncoder.getPosition());
                climberMotor.set(0);
			}
        };

        new SubsystemCommand(this.registeredCommands, "climber_down") {
			@Override
			public void initialize() {
                if(climbMode == true)
                    climberMotor.set(-1.0);
			}

			@Override
			public void execute() {
                if(climberPosition <= -85) // in rotations
                    climberMotor.set(0);
            }

			@Override
			public boolean isFinished() {
				return false;
			}

			@Override
			public void end() {
                climberMotor.set(0.0);
                System.out.println("Lifter: " + climberEncoder.getPosition());
			}
        };

        new SubsystemCommand(this.registeredCommands, "get_climber_positions") {
			@Override
			public void initialize() {
                System.out.println("Lifter: " + climberEncoder.getPosition());
			}

			@Override
			public void execute() {}

			@Override
			public boolean isFinished() {
				return false;
			}

			@Override
			public void end() {}
        };

        new SubsystemCommand(this.registeredCommands, "halt_climb") {

			@Override
			public void initialize() {
                climberMotor.set(0.0);
			}

			@Override
			public void execute() {}

			@Override
			public boolean isFinished() {
				return true;
			}

			@Override
			public void end() {}
		};
    }

	@Override
	public void init() {
        climberMotor.setSmartCurrentLimit(40);

        climberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        climberEncoder.setPosition(0);

        timeAtPress = 0;
        climbMode = false;
	}

	@Override
	public void destruct() {
        climberMotor.set(0.0);
	}
}