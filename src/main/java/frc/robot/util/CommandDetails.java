package frc.robot.util;

public class CommandDetails {

    /**
     * Enum list of type of commands available
     */
    public enum CommandType {
        // Runs at a time independant of other commands
        PARALLEL,
        
        // Runs after previous command ends
        SERIES,
        
        // Runs a parallel command (for now) after a delay specified
        TIMEDELAY
    }

    // Enum instance variable
    private CommandType commandType = null;

    // Name of command 
    private String commandName;
    
    // Arguments of the command specified
    private String commandArgs = "";

    // Time to wait before execution of the command
    private double timeDelay;

    /**
     * Constructor that splits the input and parcels off information
     * @param commandInput String input containing the name and arguments for a command
     */
    public CommandDetails(String commandInput) {
        //Splits commandInput String into an array when spaces are found
        String[] commandParts = commandInput.split(" ");

        // Sets commandName as the first string found in the commandInput
        this.commandName = commandParts[0];

        if (commandParts.length == 1) {
            return;
        }

        switch (commandParts[1]) {
            case "-s":
                this.commandType = CommandType.SERIES;
                if (commandParts.length == 3) {
                    this.commandArgs = commandParts[2];
                }
                break;
            case "-p":
                this.commandType = CommandType.PARALLEL;
                if (commandParts.length == 3) {
                    this.commandArgs = commandParts[2];
                }
                break;
            case "-t":
                this.commandType = CommandType.TIMEDELAY;
                this.timeDelay = Double.parseDouble(commandParts[2]);
                if (commandParts.length == 4) {
                    this.commandArgs = commandParts[3];
                }
                break;
            default:
                System.out.println("U DUN MESSED UP, here's what I saw as commandType = " + commandParts[1]);
                break;
        }
    }

    /**
     * @return returns the currentTime delay set
     */
    public double getDelay() {
        return this.timeDelay;
    }

    /**
     * @return Gets command type from enum
     */
    public CommandType type() {
        return this.commandType;
    }

    /**
     * @return returns command name currently in play
     */
    public String name() {
        return this.commandName;
    }

    /**
     * @return returns arguemnts for command
     */
    public String args() {
        return this.commandArgs;
    }

    /**
     * Inbuilt function to return all information on the current state of the class
     */
    @Override
    public String toString() {
        return "Command Name " + this.commandName + " Command Type " + this.commandType + " Command Args "
                + this.commandArgs;
    }
}