package frc.robot.util;

import java.util.ArrayList;
import java.util.HashMap;

import frc.robot.OI;

public class ControllerCollection extends Thread {

	private double periodNanoseconds = 0;
	private boolean stopProcessor = false;

	private HashMap<String, SubsystemModule> Controllers = new HashMap<String, SubsystemModule>();
	private ArrayList<CommandDetails> commandQueue = new ArrayList<CommandDetails>();
	private ArrayList<String> runningCommands = new ArrayList<String>();

	private int commandDivider;
	private int counter = 0;

	// Initialize controller collection with a period
	public ControllerCollection(double periodNanoseconds, int commandDivider) {
		this.periodNanoseconds = periodNanoseconds;
		this.commandDivider = commandDivider;
	}

	// Function to add the subsystem into the collection
	public void registerController(String name, SubsystemModule subsystem) {
		Controllers.put(name, subsystem);
	}

	// Thread "run" function to process all sideband controls
	public void run() {
		double timestamp_ = System.nanoTime();

		while (true) {

			if (!stopProcessor) {
				Controllers.forEach((k, v) -> v.run());

				if (counter % this.commandDivider == 0) {
					Controllers.forEach((k, v) -> v.runCommands());
				}
				counter++;

				controlInterface.checkButtons();
				commandQueue();

				while (System.nanoTime() < timestamp_ + periodNanoseconds) {

				}

				timestamp_ = System.nanoTime();

			}
		}
	}

	/*
	  Split the command input to get the command name and pull the arguments off of
	  the command input, the args will always come last. ex: target_point -s 2,3
	  ex: target_point 2,3
	 */
	public void callCommand(CommandDetails command) {
		System.out.println("Received command: " + command.toString());

		Controllers.forEach((k, v) -> {
			SubsystemCommand foundCommand = v.registeredCommands.get(command.name());
			if (foundCommand != null && command.args().isEmpty()) {
				System.out.println("found command: " + command.name());
				if (command.type().equals(CommandDetails.CommandType.TIMEDELAY)) {
					foundCommand.configureDelay(command.getDelay());
				}
				foundCommand.call();
			} else if (foundCommand != null && !command.args().isEmpty()) {
				System.out.println("found command: " + command.name());
				if (command.type().equals(CommandDetails.CommandType.TIMEDELAY)) {
					foundCommand.configureDelay(command.getDelay());
				}
				foundCommand.call(command.args());
			}
		});
	}

	// Cancel a command
	public void cancelCommand(CommandDetails command) {
		Controllers.forEach((k, v) -> {
			SubsystemCommand foundCommand = v.registeredCommands.get(command.name());
			if (foundCommand != null) {
				foundCommand.cancel();
			}
		});
	}

	// Cancels all commands
	public void cancelAll() {
		Controllers.forEach((k, v) -> {
			v.registeredCommands.forEach((k1, v1) -> {
				v1.cancel();
			});
		});
	}

	public void commandQueue() {
		/*
		  Pull new command off of queue, if it is a sequential command, make sure
		  nothing else is running. If it is a parallel command, call it and add it to
		  the list.
		 */

		if (this.commandQueue.size() > 0
				&& this.commandQueue.get(0).type().equals(CommandDetails.CommandType.PARALLEL)) {
			System.out.println(this.commandQueue.get(0).name());
			callCommand(this.commandQueue.get(0));
			this.commandQueue.remove(0);
		}

		// Check for any running commands... and EXIT
		if (this.runningCommands != null) {
			this.runningCommands.forEach(i -> {
				Controllers.forEach((k, v) -> {
					SubsystemCommand foundCommand = v.registeredCommands.get(i);
					if (foundCommand != null) {
						if (foundCommand.running) {
							return;
						}
					}
				});
			});
		}

		/*
		  If it is a sequential command, we can clear the list aaaaaaand then we add
		  the next sequential command.
		 */

		if (this.commandQueue.size() > 0 && (this.commandQueue.get(0).type().equals(CommandDetails.CommandType.SERIES)
				|| this.commandQueue.get(0).type().equals(CommandDetails.CommandType.TIMEDELAY))) {
			System.out.println(this.commandQueue.get(0).name());
			callCommand(this.commandQueue.get(0));
			this.commandQueue.remove(0);
		}

	}

	public void enable() {
		stopProcessor = false;
	}

	public void disable() {
		stopProcessor = true;
	}
}