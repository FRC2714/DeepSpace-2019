package frc.robot.util;

import java.util.ArrayList;
import frc.robot.util.CommandDetails;
import frc.robot.Robot;

public class AutoTaskGroup {

    private ArrayList<CommandDetails> commands = new ArrayList<CommandDetails>();

    public AutoTaskGroup() {

    }

    public void registerTasks () {

    }

    public void addToQueue (String command) {

        commands.add(new CommandDetails(command));

    }

    public void run () {

        for (CommandDetails v : commands) {
            Robot.ControlsProcessor.commandQueue.add(v);
        }

    }

}