package frc.robot.util;

import java.util.ArrayList;

import frc.robot.Robot;
import frc.robot.util.CommandDetails;

public class AutonTask {

    private ArrayList<CommandDetails> subtasks = new ArrayList<CommandDetails>();

    public void queueTask(String command) {
        subtasks.add(new CommandDetails(command));
    }

    public void run() {
        for (CommandDetails v : subtasks) {
            Robot.ControlsProcessor.commandQueue.add(v);
        }
    }

}
