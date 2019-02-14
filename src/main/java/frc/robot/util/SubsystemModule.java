package frc.robot.util;

import java.util.HashMap;

import edu.wpi.first.wpilibj.command.Subsystem;

// Subsystem with a hashmap of commands and a runnable
public abstract class SubsystemModule extends Subsystem {

	private boolean enabled = false;

	/**
	 * A hashmap of the different actions the subsystem can perform
	 * The key for the hashmap is the name of the command
	 */
	protected HashMap<String, SubsystemCommand> registeredCommands = new HashMap<String, SubsystemCommand>();

	/**
	 * Never gets used
	 */
	@Override
	protected void initDefaultCommand() { }

	/**
	 * Periodically called within the subsystem by the controls processor
	 */
	public abstract void run();

	/**
	 * Empty function that has to get called in constructor
	 * This is where all the subsystem commands get created
	 */ 
	public abstract void registerCommands();

	/**
	 * Gets called when the subsystem starts
	 */
	public abstract void init();

	/**
	 * Gets called when the subsystem stops
	 */
	public abstract void destruct();

	public void enable() {
		enabled = true;
	}

	public void disable() {
		enabled = false;
	}

	public boolean getStatus() {
		return enabled;
	}

	/**
	 * Gets called when the run function gets called
	 * Acts like the WPILIB commands
	 */
	public void runCommands() {

		// Loops through every value in the hashmap
		registeredCommands.forEach((k, v) -> {

			// Call the initializer if the first run is active
			if (v.firstRun && v.checkDelayExpired()) {
				v.firstRun = false;
				v.running = true;
			}

			// Call the execute function if the command is still active
			if (v.running) {
				// If the command is finished, exit
				if (v.isFinished()) {
					System.out.println("Command isFinished == true");
					v.end();
					v.running = false;
				}
				
				v.execute();
			}
		});

	}

}
