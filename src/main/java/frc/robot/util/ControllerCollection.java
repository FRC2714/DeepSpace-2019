package frc.robot.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.io.*;
import frc.robot.Robot;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.SelectionKey;
import java.nio.channels.Selector;
import java.nio.channels.ServerSocketChannel;
import java.nio.channels.SocketChannel;
import java.util.Iterator;
import java.util.Set;

public class ControllerCollection extends Thread {

	public double periodNanoseconds = 0;
	public boolean stopProcessor = false;

	Selector selector;
	ArrayList<CommandDetails> commandQueue = new ArrayList<CommandDetails>();

	public ArrayList<String> runningCommands = new ArrayList<String>();
	public String mainCommand;

	public HashMap<String, SubsystemModule> Controllers = new HashMap<String, SubsystemModule>();

	private int commandDivider;
	int counter = 0;

	// Initialize controller collection with a period
	public ControllerCollection(double periodNanoseconds, int commandDivider) {
		this.periodNanoseconds = periodNanoseconds;
		this.commandDivider = commandDivider;

		try {
			int port = 19000;
			this.selector = Selector.open();
			ServerSocketChannel ssChannel = ServerSocketChannel.open();
			ssChannel.configureBlocking(false);
			ssChannel.socket().bind(new InetSocketAddress(port));
			ssChannel.register(selector, SelectionKey.OP_ACCEPT);
		} catch (Exception e) {

		}

	}

	// Stop the controller
	public void stopProcessor() {
		stopProcessor = true;
	}

	public void processReadySet(Set readySet) throws Exception {
		Iterator iterator = readySet.iterator();
		while (iterator.hasNext()) {
			SelectionKey key = (SelectionKey) iterator.next();
			iterator.remove();
			if (key.isAcceptable()) {
				ServerSocketChannel ssChannel = (ServerSocketChannel) key.channel();
				SocketChannel sChannel = (SocketChannel) ssChannel.accept();
				sChannel.configureBlocking(false);
				sChannel.register(key.selector(), SelectionKey.OP_READ);
			}
			if (key.isReadable()) {
				String msg = processRead(key);
				if (msg.length() > 0) {
					commandQueue.add(new CommandDetails(msg));
				}
			}
		}
	}

	public String processRead(SelectionKey key) throws Exception {
		SocketChannel sChannel = (SocketChannel) key.channel();
		ByteBuffer buffer = ByteBuffer.allocate(1024);
		int bytesCount = sChannel.read(buffer);
		if (bytesCount > 0) {
			buffer.flip();
			return new String(buffer.array());
		}
		return "";
	}

	// Function to add the subsystem into the collection
	public void registerController(String name, SubsystemModule subsystem) {
		Controllers.put(name, subsystem);
	}

	// Thread "run" function, uses busy wait to get under Java 1ms limit
	public void run() {
		double timestamp_ = System.nanoTime();

		while (true) {

			if (!stopProcessor){
				Controllers.forEach((k, v) -> v.run());

				if (counter % this.commandDivider == 0) {
					Controllers.forEach((k, v) -> v.runCommands());
				}
				counter++;

				// Process buttons
				Robot.oi.buttons.forEach((k) -> k.checkButton());

				try {
					if (selector.selectNow() > 0) {
						processReadySet(selector.selectedKeys());
					}
				} catch (IOException e) {
					e.printStackTrace();
				} catch (Exception e) {
					e.printStackTrace();
				}

				commandQueue();

				while (System.nanoTime() < timestamp_ + periodNanoseconds) {

				}

				timestamp_ = System.nanoTime();

			}
		}
	}

	/*
	Split the command input to get the command name and pull the arguments off of the command input, 
	the args will always come last.
	ex: target_point -s 2,3
	ex: target_point 2,3
	*/
	public void callCommand(CommandDetails command) {
		System.out.println("Received command: " + command.toString());

		Controllers.forEach((k, v) -> {
			SubsystemCommand foundCommand = v.registeredCommands.get(command.name());
			if (foundCommand != null && command.args().isEmpty()) {
				System.out.println("found command: " + command.name());
				if (command.type().equals(CommandDetails.CommandType.TIMEDELAY)){
					foundCommand.configureDelay(command.getDelay());
				}
				foundCommand.call();
			} else if (foundCommand != null && !command.args().isEmpty()) {
				System.out.println("found command: " + command.name());
				if (command.type().equals(CommandDetails.CommandType.TIMEDELAY)){
					foundCommand.configureDelay(command.getDelay());
				}
				foundCommand.call(command.args());
			}
		});
	}

	//cancel a command
	public void cancelCommand(CommandDetails command) {
		Controllers.forEach((k, v) -> {
			SubsystemCommand foundCommand = v.registeredCommands.get(command.name());
			if (foundCommand != null) {
				foundCommand.cancel();
			}
		});
	}

	public void cancelAll() {
		Controllers.forEach((k, v) -> {
			v.registeredCommands.forEach((k1, v1) -> {
				v1.cancel();
			});
		});
	}

	public void commandQueue() {
		/*
		 * Pull new command off of queue, if it is a sequential command, make sure
		 * nothing else is running. If it is a parallel command, call it and add it to
		 * the list.
		 */

		if (this.commandQueue.size() > 0 && this.commandQueue.get(0).type().equals(CommandDetails.CommandType.PARALLEL)) {
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
		 * If it is a sequential command, we can clear the list aaaaaaand then we add
		 * the next sequential command.
		 */

		if (this.commandQueue.size() > 0 && (this.commandQueue.get(0).type().equals(CommandDetails.CommandType.SERIES) || this.commandQueue.get(0).type().equals(CommandDetails.CommandType.TIMEDELAY))){
			System.out.println(this.commandQueue.get(0).name());
			callCommand(this.commandQueue.get(0));
			this.commandQueue.remove(0);
		}

	}
}
