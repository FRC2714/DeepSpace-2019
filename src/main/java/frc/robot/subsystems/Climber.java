package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.util.SubsystemCommand;
import frc.robot.util.SubsystemModule;

public class Climber extends SubsystemModule {

    private CANSparkMax climberMotor = new CANSparkMax(11, MotorType.kBrushless);
    private CANSparkMax climberAssistor = new CANSparkMax(12,MotorType.kBrushless);
    // private CANSparkMax climberPump = new CANSparkMax(12, MotorType.kBrushed);

    // MAX encoders
	private CANEncoder climberEncoder = climberMotor.getEncoder();

	private Servo climberValve = new Servo(2);

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
	                System.out.println("Giving Power 1.0 to climber");
                    climberMotor.set(1);
                    climberAssistor.set(1);
                    climbMode = true;
			}

			@Override
			public void execute() {
//				if(climberEncoder.getPosition() >= 220) { // in rotations
//					System.out.println("Stopping Climber Motor");
//					climberMotor.set(0.0);
//					climberAssistor.set(0.0);
//				}
				climberValve.set(0.0);
			}

			@Override
			public boolean isFinished() {
            	//Actual highest position = 235
				return climberEncoder.getPosition() >= 240;
			}

			@Override
			public void end() {
				System.out.println("Ending upper climber motor movement");
                System.out.println("Lifter: " + climberEncoder.getPosition());
                climberMotor.set(0.0);
                climberAssistor.set(0.0);
			}
        };

        new SubsystemCommand(this.registeredCommands, "climber_down") {
			@Override
			public void initialize() {

				climberMotor.set(-1);
				climberAssistor.set(-1);

			}

			@Override
			public void execute() {
//				if(climberEncoder.getPosition() <= -90) { // in rotations
//					climberMotor.set(0);
//					climberAssistor.set(0);
//				}
            }

			@Override
			public boolean isFinished() {
				//Actual Lowest = -90
				return climberEncoder.getPosition() <= -80;
			}

			@Override
			public void end() {
				System.out.println("Ending downward climber motor movement");
				climberMotor.set(0.0);
				climberAssistor.set(0.0);
                System.out.println("Lifter: " + climberEncoder.getPosition());
			}
        };

	    new SubsystemCommand(this.registeredCommands, "valve_off") {
	    	double initTime;
		    @Override
		    public void initialize() {
		    	initTime = System.nanoTime();
		    }

		    @Override
		    public void execute() {
			    climberValve.set(1);
			    System.out.println("Valve Setting to 1");
		    }

		    @Override
		    public boolean isFinished() {
			    return (System.nanoTime() - initTime) > 1000000000;
		    }

		    @Override
		    public void end() {

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
                climberAssistor.set(0.0);
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

        new SubsystemCommand(this.registeredCommands, "print_climber_position"){
	        @Override
	        public void initialize() {
		        System.out.println("Current Climber Encoder = " + climberEncoder.getPosition());
	        }

	        @Override
	        public boolean isFinished() {
		        return true;
	        }
        };
    }



	@Override
	public void init() {
        climberMotor.setSmartCurrentLimit(60);
        climberAssistor.setSmartCurrentLimit(60);

        //MAKE BRAKE AFTER TUNING
        climberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        climberAssistor.setIdleMode(CANSparkMax.IdleMode.kBrake);

		climberAssistor.setInverted(true);
        climberEncoder.setPosition(0);

        timeAtPress = 0;
        climbMode = false;
	}

	@Override
	public void destruct() {
        climberMotor.set(0.0);
        climberAssistor.set(0.0);
	}
}