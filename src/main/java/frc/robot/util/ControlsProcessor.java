package frc.robot.util;

import java.util.ArrayList;
import java.util.HashMap;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.RobotMap;

import frc.robot.util.WebsocketButtonPad;

import java.net.URI;

public abstract class ControlsProcessor extends Thread {

	private double periodNanoseconds = 0;
	private boolean stopProcessor = false;

	private HashMap<String, SubsystemModule> controllers = new HashMap<String, SubsystemModule>();
	private ArrayList<CommandDetails> commandQueue = new ArrayList<CommandDetails>();

	private int commandDivider;
	private int counter = 0;

	// Controllers and button boxes
	protected Joystick xbox1 = new Joystick(RobotMap.p_xbox1);
	protected Joystick newButtonBoxA = new Joystick(RobotMap.p_newButtonBoxA);
	protected Joystick newButtonBoxB = new Joystick(RobotMap.p_newButtonBoxB);
	protected Joystick buttonBoxA = new Joystick(RobotMap.p_buttonBox);

	// xbox1 buttons
	protected JoystickButton a = new JoystickButton(xbox1, 1);
	protected JoystickButton b = new JoystickButton(xbox1, 2);
	protected JoystickButton x = new JoystickButton(xbox1, 3);
	protected JoystickButton y = new JoystickButton(xbox1, 4);
	protected JoystickButton lb = new JoystickButton(xbox1, 5);
	protected JoystickButton rb = new JoystickButton(xbox1, 6);
	protected JoystickButton back = new JoystickButton(xbox1, 7);
	protected JoystickButton start = new JoystickButton(xbox1, 8);
	protected JoystickButton leftStick = new JoystickButton(xbox1, 9);
	protected JoystickButton rightStick = new JoystickButton(xbox1, 10);

	protected WebsocketButtonPad launchpad;

	// Array list that holds all of the operator controls
	private ArrayList<JoystickCommandPair> operatorControls = new ArrayList<JoystickCommandPair>();
	
	/**
	 * Has to be overridden from robot class
	 */
	public abstract void registerOperatorControls();

	/**
	 * Initialize controller collection with specified period
	 * @param periodNanoseconds Period in nanoseconds
	 * @param commandDivider How often the commands get processed alongside the actual control loop
	 */
	public ControlsProcessor(double periodNanoseconds, int commandDivider) {
		this.periodNanoseconds = periodNanoseconds;
		this.commandDivider = commandDivider;
		
		try {
			launchpad = new WebsocketButtonPad( new URI( "ws://10.27.14.5:5802" )); // ws://10.27.14.207:5802 on driver station
			launchpad.connect();
		} catch (Exception e) {
			System.out.println("Websocket failure");
		}

		registerOperatorControls();
	}

	public void connectButtonPad(){
		if(!launchpad.isOpen()){
			try {
				launchpad.connect();
			} catch (Exception e) {
				System.out.println("Websocket connection failure");
			}
		}
		else {
			System.out.println("Websocket connected");
		}	

	}
	/**
	 * Function to add the subsystem into the collection
	 * All the runs are added and called periodically
	 * @param name 
	 * @param subsystem
	 */
	public void registerController(String name, SubsystemModule subsystem) {
		controllers.put(name, subsystem);
	}

	/**
	 * Runs periodically as required by extension of Java Thread
	 */
	public void run() {

		connectButtonPad();
		double timestamp = System.nanoTime();

		// Runs even when robot is disabled
		while (true) {

			if (!stopProcessor) {
				timestamp = System.nanoTime();

				controllers.forEach((k, v) -> v.run());

				if (counter % this.commandDivider == 0) {
					controllers.forEach((k, v) -> v.runCommands());
				}
				counter++;
				
				checkButtons();
				processCommandQueue();
				
				// Busy wait until the next iteration
				while (System.nanoTime() < timestamp + periodNanoseconds) { }
				
			}
		}
	}

	/**
	 * Split the command input to get the command name and pull the arguments off of 
	 * the command input, the args will always come last. 
	 * ex: target_point -s 2,3ex: target_point 2,3
	 * 
	 * Called in commandQueue
	 * @param command Command in reference
	 */
	public void callCommand(CommandDetails command) {
		System.out.println("Received command: " + command.toString());
		
		controllers.forEach((k, v) -> {

			// Matches the name of registered commands in the subsystem constructors
			SubsystemCommand foundCommand = v.registeredCommands.get(command.name());

			// Distinguishes based on whether or not the command contains arguments
			if (foundCommand != null && command.args().isEmpty()) {
				System.out.println("Found command: " + command.name());

				if (command.type().equals(CommandDetails.CommandType.TIMEDELAY)) {
					foundCommand.configureDelay(command.getDelay());
				}

				foundCommand.call();

			} else if (foundCommand != null && !command.args().isEmpty()) {
				System.out.println("Found command: " + command.name());

				if (command.type().equals(CommandDetails.CommandType.TIMEDELAY)) {
					foundCommand.configureDelay(command.getDelay());
				}

				foundCommand.call(command.args());
			}
		});
	}

	// Cancel a command based on input
	public void cancelCommand(CommandDetails command) {
		controllers.forEach((k, v) -> {
			SubsystemCommand foundCommand = v.registeredCommands.get(command.name());

			System.out.println("Cancelling Command = " + command.name());

			if (foundCommand != null) {
				foundCommand.cancel();
			}
		});
	}

	// Cancels all commands running
	public void cancelAll() {
		commandQueue = new ArrayList<CommandDetails>(0);
		controllers.forEach((k, v) -> {
			v.registeredCommands.forEach((k1, v1) -> {
				if(v1.running) {
					System.out.println("Cancelling Command :- " + v1.getName());
					v1.cancel();
				}
			});
		});
	}

	/**
	 * Adds a command to commandQueue
	 */
	public void addToQueue(CommandDetails newCommand) {
		commandQueue.add(newCommand);
	}

	/**
	 * Pull new command off of queue, if it is a sequential command, make sure
	 * nothing else is running. If it is a parallel command, call it and add it to
	 * the list.
	 */
	public void processCommandQueue() {
		// TODO: Test time delay
		if (this.commandQueue.size() > 0 && (this.commandQueue.get(0).type().equals(CommandDetails.CommandType.PARALLEL)
				|| this.commandQueue.get(0).type().equals(CommandDetails.CommandType.TIMEDELAY))) {
			System.out.println("parallel" + this.commandQueue.get(0).name());
			callCommand(this.commandQueue.get(0));
			System.out.println("Calling Command : " + this.commandQueue.get(0));
			this.commandQueue.remove(0);
		}

		// Checks to see if there are any commands currently running, and if there, it exits the method
		// This would prevent any sequential commands from running
		for(java.util.Map.Entry<String, SubsystemModule> sub : controllers.entrySet()) {
			for (java.util.Map.Entry<String, SubsystemCommand> cmds : sub.getValue().registeredCommands.entrySet()) {
				if (cmds.getValue().running) {
					return;
				}
			}
		}

		// controllers.forEach((k, v) -> {
		// 	v.registeredCommands.forEach((a,b) -> {
		// 		if (b.running){
		// 			return;
		// 		}

		// 	});
		// });

		/**
		 * If it is a sequential command, we can clear the list and then we add
		 * the next sequential command.
		 */
		if (this.commandQueue.size() > 0 && (this.commandQueue.get(0).type().equals(CommandDetails.CommandType.SERIES))) {
			System.out.println("series" + this.commandQueue.get(0).name());
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
		operatorControls.add(new JoystickCommandPair(this, command, button));
    }

	/**
	 * Calls checkButton for every button on controller
	 */
    public void checkButtons() {
		operatorControls.forEach((k) -> k.checkButton());
    }

	/**
	 * @return Returns left Joystick
	 */
	public double getLeftJoystick() {
		return xbox1.getRawAxis(1);
	}

	/**
	 * @return Returns right Joystick
	 */
	public double getRightJoystick() {
		return xbox1.getRawAxis(4);
	}

	/**
	 * @return Returns the command period with commandDivider in seconds
	 */
	public double getCommandPeriod() {
		return (this.periodNanoseconds / 1000000000) * commandDivider;
	}

	/**
	 * 
	 * @return Returns the period in seconds
	 */
	public double getControlsPeriod() {
		return (this.periodNanoseconds / 1000000000);
	}
}