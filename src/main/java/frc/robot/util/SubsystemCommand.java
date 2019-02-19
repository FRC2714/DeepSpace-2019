package frc.robot.util;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Timer;

public abstract class SubsystemCommand {

    // Store the name of the command and the number of parameters passed into the command
    private String commandName;

    protected boolean firstRun = false;
    protected boolean running = false;

    private double delay = 0; // TODO: Test
    private Timer delayTimer;

    protected String[] args;

    /**
     * Constructor for the command
     * @param commands Hashmap to add itself onto
     * @param commandName The name of the command
     */
    public SubsystemCommand(HashMap<String, SubsystemCommand> commands, String commandName) {
        this.commandName = commandName;
        this.delayTimer = new Timer();
        commands.put(this.commandName, this);
    }

    /**
     * TODO: Test
     * @param delay Delay in seconds before calling a command
     */
    public void configureDelay(double delay) {
        this.delay = delay;

        if (this.delay > 0) {
            this.delayTimer.start();
        }

    }

    /**
     * Gets called instead of execute until the delay expires
     * @return True if delay is expired
     */
    public boolean checkDelayExpired() {
        if (this.delay == 0) {
            return true;
        }
        if (this.delayTimer.get() > this.delay) {
            this.delayTimer.stop();
            this.delayTimer.reset();
            return true;
        } else {
            return false;
        }
    }

    /**
     * Starts the command
     * @param parameters Parameters for the command
     */
    public void call(String parameters) {
        this.firstRun = true;
        this.args = parameters.split(",");

        if (this.delay > 0) {
            this.delayTimer.start();
        }
        this.running = true;
        initialize();
    }

    /**
     * Starts the command
     */
    public void call() {
        this.firstRun = true;
        this.running = true;
        initialize();
    }

    /**
     * Cancels the command and calls end
     */
    public void cancel() {
        end();
        
        this.running = false;
    }

    /**
     * Called once when the command is called
     */
    public void initialize() {

    }

    /**
     * Called once each iteration of the subsystem module
     */
    public void execute() {

    }

    /**
     * Redefine to check if the condition for exit is met
     * @return True if finished
     */
    public boolean isFinished() {
        return true;
    }

    /**
     * Called once at the end of the command
     */
    public void end() {

    }
}