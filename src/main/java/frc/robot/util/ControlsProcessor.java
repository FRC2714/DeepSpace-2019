package frc.robot.util;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.RobotMap;

public class ControlsProcessor extends Thread {

	private double periodNanoseconds = 0;
	private boolean stopProcessor = false;

	private HashMap<String, SubsystemModule> controllers = new HashMap<String, SubsystemModule>();
	private ArrayList<CommandDetails> commandQueue = new ArrayList<CommandDetails>();
	private ArrayList<String> runningCommands = new ArrayList<String>();

	private int commandDivider;
	private int counter = 0;

	// Controllers and button boxes
	Joystick xbox1 = new Joystick(RobotMap.p_xbox1);
	Joystick newButtonBoxA = new Joystick(RobotMap.p_newButtonBoxA);
	Joystick newButtonBoxB = new Joystick(RobotMap.p_newButtonBoxB);
	Joystick buttonBoxA = new Joystick(RobotMap.p_buttonBox);

	// xbox1 buttons
	JoystickButton a = new JoystickButton(xbox1, 1);
	JoystickButton b = new JoystickButton(xbox1, 2);
	JoystickButton x = new JoystickButton(xbox1, 3);
	JoystickButton y = new JoystickButton(xbox1, 4);
	JoystickButton lb = new JoystickButton(xbox1, 5);
	JoystickButton rb = new JoystickButton(xbox1, 6);
	JoystickButton back = new JoystickButton(xbox1, 7);
	JoystickButton start = new JoystickButton(xbox1, 8);

	// buttonBoxA buttons
	JoystickButton bb11 = new JoystickButton(buttonBoxA, 16);
	JoystickButton bb12 = new JoystickButton(buttonBoxA, 10);
	JoystickButton bb13 = new JoystickButton(buttonBoxA, 15);
	JoystickButton bb14 = new JoystickButton(buttonBoxA, 13);
	JoystickButton bb15 = new JoystickButton(buttonBoxA, 14);
	JoystickButton bb16 = new JoystickButton(buttonBoxA, 5);
	JoystickButton bb17 = new JoystickButton(buttonBoxA, 11);
	JoystickButton bb18 = new JoystickButton(buttonBoxA, 12);
	JoystickButton bb19 = new JoystickButton(buttonBoxA, 3);
	JoystickButton bb110 = new JoystickButton(buttonBoxA, 4);
	JoystickButton bb111 = new JoystickButton(buttonBoxA, 8);
	JoystickButton bb112 = new JoystickButton(buttonBoxA, 7);
	JoystickButton bb113 = new JoystickButton(buttonBoxA, 9);

	// newButtonBoxA
	JoystickButton nbba1 = new JoystickButton(newButtonBoxA, 1);
	JoystickButton nbba2 = new JoystickButton(newButtonBoxA, 2);
	JoystickButton nbba3 = new JoystickButton(newButtonBoxA, 3);
	JoystickButton nbba4 = new JoystickButton(newButtonBoxA, 4);
	JoystickButton nbba5 = new JoystickButton(newButtonBoxA, 5);
	JoystickButton nbba6 = new JoystickButton(newButtonBoxA, 12);
	JoystickButton nbba7 = new JoystickButton(newButtonBoxA, 11);
	JoystickButton nbba8 = new JoystickButton(newButtonBoxA, 6);
	JoystickButton nbba9 = new JoystickButton(newButtonBoxA, 7);
	JoystickButton nbba10 = new JoystickButton(newButtonBoxA, 8);
	JoystickButton nbba11 = new JoystickButton(newButtonBoxA, 9);
	JoystickButton nbba12 = new JoystickButton(newButtonBoxA, 10);

	// newButtonBoxB
	JoystickButton nbbb1 = new JoystickButton(newButtonBoxB, 1);
	JoystickButton nbbb2 = new JoystickButton(newButtonBoxB, 2);
	JoystickButton nbbb3 = new JoystickButton(newButtonBoxB, 3);
	JoystickButton nbbb4 = new JoystickButton(newButtonBoxB, 11);
	JoystickButton nbbb5 = new JoystickButton(newButtonBoxB, 10);
	JoystickButton nbbb6 = new JoystickButton(newButtonBoxB, 9);
	JoystickButton nbbb7 = new JoystickButton(newButtonBoxB, 8);
	JoystickButton nbbb8 = new JoystickButton(newButtonBoxB, 7);
	JoystickButton nbbb9 = new JoystickButton(newButtonBoxB, 6);
	JoystickButton nbbb10 = new JoystickButton(newButtonBoxB, 5);
	JoystickButton nbbb11 = new JoystickButton(newButtonBoxB, 4);
	JoystickButton nbbb12 = new JoystickButton(newButtonBoxB, 3);
	JoystickButton nbbb13 = new JoystickButton(newButtonBoxB, 2);
	JoystickButton nbbb14 = new JoystickButton(newButtonBoxB, 1);

	private ArrayList<JoystickCommandPair> controls;
	private ControlsProcessor cc_reference;
	
	// Override this from robot class
	public void registerOperatorControls() {

	}

	// Initialize controller collection with a period
	public ControlsProcessor(double periodNanoseconds, int commandDivider) {
		this.periodNanoseconds = periodNanoseconds;
		this.commandDivider = commandDivider;
	}

	// Function to add the subsystem into the collection
	public void registerController(String name, SubsystemModule subsystem) {
		controllers.put(name, subsystem);
	}

	// Thread "run" function to process all sideband controls
	public void run() {
		double timestamp_ = System.nanoTime();

		while (true) {

			if (!stopProcessor) {
				controllers.forEach((k, v) -> v.run());

				if (counter % this.commandDivider == 0) {
					controllers.forEach((k, v) -> v.runCommands());
				}
				counter++;

				checkButtons();
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

		controllers.forEach((k, v) -> {
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
		controllers.forEach((k, v) -> {
			SubsystemCommand foundCommand = v.registeredCommands.get(command.name());
			if (foundCommand != null) {
				foundCommand.cancel();
			}
		});
	}

	// Cancels all commands
	public void cancelAll() {
		controllers.forEach((k, v) -> {
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
				controllers.forEach((k, v) -> {
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

	// Append to the registered buttons and commands
    public void append(String command, JoystickButton button) {
        controls.add(new JoystickCommandPair(this.cc_reference, command, button));
    }

    public void checkButtons() {
		controls.forEach((k) -> k.checkButton());
    }

	public double getLeftJoystick() {
		return xbox1.getRawAxis(1);
	}

	public double getRightJoystick() {
		return xbox1.getRawAxis(4);
	}
}