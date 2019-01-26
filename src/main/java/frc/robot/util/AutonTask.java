package frc.robot.util;

import java.util.ArrayList;
import frc.robot.util.CommandDetails;

public class AutonTask {

    private ArrayList<CommandDetails> subtasks = new ArrayList<CommandDetails>();
    private ControlsProcessor controlsProcessor;
    
    /**
     * Accepts the object of the running controlsProcessor to modify
     * @param controlsProcessor
     */
    public AutonTask(ControlsProcessor controlsProcessor) {
        this.controlsProcessor = controlsProcessor;
    }

    /**
     * Adds the command to the list of subtasks
     * @param command
     */
    public void queueTask(String command) {
        subtasks.add(new CommandDetails(command));
    }

    /**
     * Adds all subtasks to actual commandQueue
     */
    public void run() {
        for (CommandDetails v : subtasks) {
            controlsProcessor.addToQueue(v);
        }
    }

}
