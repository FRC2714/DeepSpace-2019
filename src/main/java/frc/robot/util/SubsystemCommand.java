package frc.robot.util;

import java.util.HashMap;

public abstract class SubsystemCommand {

    // Store the name of the command and the number of parameters passed into the command
    private String commandName;

    public boolean firstRun = false;
    public boolean running = false;

    public String[] args;

    // Constructor, set up the command
    public SubsystemCommand (HashMap<String, SubsystemCommand> commands, String commandName) {
        this.commandName = commandName;

        commands.put(this.commandName, this);
    }

    public void call (String parameters) {
        this.firstRun = true;
        this.args = parameters.split(",");
    }

    public void call () {
        this.firstRun = true;
    }

    public void cancel() {
        end();

        this.running = false;
    }

    // Called once at the beginning
    public void initialize () {

    }

    // Called once each iteration of the subsystem module
    public void execute () {

    }

    // Redefine to check if the condition for exit is met
    public boolean isFinished () {
        return true;
    }

    // Called once at the end of the command
    public void end () {

    }

}