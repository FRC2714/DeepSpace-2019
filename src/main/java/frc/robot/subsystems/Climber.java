package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.util.SubsystemCommand;
import frc.robot.util.SubsystemModule;

public class Climber extends SubsystemModule {

    private CANSparkMax lifterMotor = new CANSparkMax(11, MotorType.kBrushless);
    private CANSparkMax pusherMotor = new CANSparkMax(12, MotorType.kBrushless);

    // MAX encoders
	private CANEncoder lifterEncoder = lifterMotor.getEncoder();
    private CANEncoder pusherEncoder = pusherMotor.getEncoder();
    
    // Motor to output ratios
    private final double lifterRatio = 1; //TODO: Get real value
    private final double pusherRatio = 1; //TODO: Get real value

    // Climber positions
    private double lifterPosition;
    private double pusherPosition;

    // 
    private boolean lifterDownDone;
    private boolean lifterUpDone;
    private boolean pusherOutDone;
    private boolean pusherInDone;

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
        lifterPosition = lifterEncoder.getPosition() * lifterRatio;
        pusherPosition = lifterEncoder.getPosition() * pusherRatio;
    }

    public void clearClimbState() {
        lifterDownDone = false;
        lifterUpDone = false;
        pusherOutDone = false;
        pusherInDone = false;
    }

    public void lifterDown() {
        if(!lifterDownDone && lifterPosition < 0) {
            lifterMotor.set(0.75);
        } else {
            lifterMotor.set(0.0);
            lifterDownDone = true;
        }
    }

    public void lifterUp() {
        if(!lifterUpDone && lifterPosition > 0) {
            lifterMotor.set(-0.25);
        } else {
            lifterMotor.set(0.0);
            lifterUpDone = true;
        }
    }

    public void pusherOut() {
        if(!pusherOutDone && pusherPosition < 0) {
            pusherMotor.set(-0.5);
        } else {
            pusherMotor.set(0.0);
            pusherOutDone = true;
        }
    }

    public void pusherIn() {
        if(!pusherInDone && pusherPosition > 0) {
            pusherMotor.set(0.25);
        } else {
            pusherMotor.set(0.0);
            pusherInDone = true;
        }
    }

    @Override public void run() {
        updatePositions();
    }

    @Override public void registerCommands() {

        new SubsystemCommand(this.registeredCommands, "lifter_down") {

            @Override
			public void initialize() {
                if(System.nanoTime() - timeAtPress < 1000000000 || climbMode == true) {
                    lifterMotor.set(1.0);
                    climbMode = true;
                } else {
                    timeAtPress = System.nanoTime();
                }
			}

			@Override
			public void execute() {}

			@Override
			public boolean isFinished() {
				return false;
			}

			@Override
			public void end() {
                System.out.println("Lifter: " + lifterEncoder.getPosition());
			}
        };

        new SubsystemCommand(this.registeredCommands, "lifter_up") {
			@Override
			public void initialize() {
                if(climbMode == true) {
                    lifterMotor.set(-0.5);
                }
			}

			@Override
			public void execute() {}

			@Override
			public boolean isFinished() {
				return false;
			}

			@Override
			public void end() {
                lifterMotor.set(0.0);
			}
        };

        new SubsystemCommand(this.registeredCommands, "pusher_out") {
			@Override
			public void initialize() {
                if(climbMode == true) {
                    pusherMotor.set(-0.2);
                }
			}

			@Override
			public void execute() {}

			@Override
			public boolean isFinished() {
				return true;
			}

			@Override
			public void end() {
                System.out.println("Pusher: " + pusherEncoder.getPosition());
			}
        };

        new SubsystemCommand(this.registeredCommands, "pusher_in") {
			@Override
			public void initialize() {
                if(climbMode == true) {
                    pusherMotor.set(0.2);
                }
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

        new SubsystemCommand(this.registeredCommands, "get_climber_positions") {
			@Override
			public void initialize() {
                System.out.println("Lifter: " + lifterEncoder.getPosition() + "\tPusher: " + pusherEncoder.getPosition());
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

        new SubsystemCommand(this.registeredCommands, "send_climb") {
            boolean done;

			@Override
			public void initialize() {
                done = false;
			}

			@Override
			public void execute() {
                if(!lifterDownDone) {
                    lifterDown();
                } else if(!pusherOutDone) {
                    pusherOut();
                } else if(!lifterUpDone) {
                    lifterUp();
                } else if(!pusherInDone) {
                    pusherIn();
                } else {
                    done = true;
                }
			}

			@Override
			public boolean isFinished() {
				return done;
			}

			@Override
			public void end() {
                clearClimbState();
			}
        };
        
        new SubsystemCommand(this.registeredCommands, "halt_climb") {

			@Override
			public void initialize() {
                lifterMotor.set(0.0);
                pusherMotor.set(0.0);
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
        clearClimbState();

        lifterMotor.setSmartCurrentLimit(40);
        pusherMotor.setSmartCurrentLimit(40);

        lifterMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		pusherMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        lifterEncoder.setPosition(0);
        pusherEncoder.setPosition(0);

        timeAtPress = 0;
        climbMode = false;
	}

	@Override
	public void destruct() {
        lifterMotor.set(0.0);
        pusherMotor.set(0.0);
	}
}