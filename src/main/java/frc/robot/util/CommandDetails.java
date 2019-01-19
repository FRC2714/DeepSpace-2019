package frc.robot.util;

public class CommandDetails {

    public enum CommandType {
        PARALLEL, SERIES, TIMEDELAY
    }

    private CommandType commandType = null;
    private String commandName;
    private String commandArgs = "";
    private double timeDelay;

    public CommandDetails(String commandInput) {
        String[] commandParts = commandInput.split(" ");

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
        }
    }

    public double getDelay() {
        return this.timeDelay;
    }

    public CommandType type() {
        return this.commandType;
    }

    public String name() {
        return this.commandName;
    }

    public String args() {
        return this.commandArgs;
    }

    @Override
    public String toString() {
        return "Command Name " + this.commandName + " Command Type " + this.commandType + " Command Args "
                + this.commandArgs;
    }
}