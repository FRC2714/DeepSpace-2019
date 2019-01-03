package frc.robot.util;

import java.util.HashMap;

import edu.wpi.first.wpilibj.command.Subsystem;

// Subsystem with a hashmap of commands and a runnable
public abstract class SubsystemModule extends Subsystem {

	public boolean enabled = false;
	protected HashMap<String, SubsystemCommand> registeredCommands = new HashMap<String, SubsystemCommand>();
	
	@Override
	protected void initDefaultCommand() {

	}
	
	public abstract void run();
	public abstract void registerCommands();

	public void runCommands() {

		registeredCommands.forEach((k,v) ->  {

			// Call the initializer if the first run is active
			if (v.firstRun && v.checkDelayExpired()) {
				v.initialize();
				v.firstRun = false;
				v.running = true;
			}
			// Call the execute function if the command is still active
			if (v.running) {
				v.execute();

				// If the command is finished, exit
				if (v.isFinished()) {
					v.end();
					v.running = false;
				}
			}
		});

	}

}
