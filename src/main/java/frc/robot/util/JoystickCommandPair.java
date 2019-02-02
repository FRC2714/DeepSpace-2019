package frc.robot.util;

import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class JoystickCommandPair {

    // A button from the operator interface
    private JoystickButton button;
    
    // Contains the details about the command that you are binding to the button
    private CommandDetails commandDetails;
 
    // Instance of a ControlsProcessor 
    private ControlsProcessor controlsProcessor;

    // Save previous state to call only when on edge.
    private boolean lastState = false;

    /**
     * @param controlsProcessor The current ControlsProcessor object in reference
     * @param commandInput The whole command string appended
     * @param buttonToPair The button to pair the command with
     */
    public JoystickCommandPair(ControlsProcessor controlsProcessor, String commandInput, JoystickButton buttonToPair) {
        this.button = buttonToPair;
        this.commandDetails = new CommandDetails(commandInput);
        this.controlsProcessor = controlsProcessor;
    }

    /**
     * Called periodically to check for button edges 
     */
    public void checkButton() {
        boolean currentState = this.button.get();

        if (currentState && !this.lastState) {
            controlsProcessor.callCommand(this.commandDetails);
        }

        if (!currentState && this.lastState && this.commandDetails.type() == 
            CommandDetails.CommandType.SERIES) {
            controlsProcessor.cancelCommand(this.commandDetails);
        }

        this.lastState = currentState;
    }
}