package frc.robot.util;

public class CommandDetails {

    public enum CommandType {PARALLEL, SERIES}
    private CommandType commandType = null;
    private String commandName;
    private String commandArgs = "";
    
    public CommandDetails (String commandInput) {
        String[] commandParts = commandInput.split(" ");
        
        this.commandName = commandParts[0];

        if (commandParts.length == 1){
            return;
        }
        switch (commandParts[1]){
            case "-s":
            this.commandType = CommandType.SERIES;
            break;
            case "-p":
            this.commandType = CommandType.PARALLEL;
            break;
        }

        if (commandParts.length == 3) {
            this.commandArgs = commandParts[2];
        }
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
    public String toString(){
        return "Command Name "+this.commandName+" Command Type "+ this.commandType+" Command Args "+ this.commandArgs;
    }
}