package frc.robot.util;

import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.Robot;

public class JoystickCommandPair {

    private JoystickButton thisButton;
    private CommandDetails commandDetails;

    private boolean lastState = false;

    public JoystickCommandPair(String commandInput, JoystickButton buttonToPair) {
        this.thisButton = buttonToPair;
        this.commandDetails = new CommandDetails(commandInput);
    }

    public void checkButton() {
        boolean currentState = this.thisButton.get();

        if (currentState && !this.lastState) {
            Robot.ControlsProcessor.callCommand(this.commandDetails);
        }

        if (!currentState && this.lastState && this.commandDetails.type() == CommandDetails.CommandType.SERIES) {
            Robot.ControlsProcessor.cancelCommand(this.commandDetails);
        }

        this.lastState = currentState;
    }

}