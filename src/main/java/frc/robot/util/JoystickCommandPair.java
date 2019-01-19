package frc.robot.util;

import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class JoystickCommandPair {

    private JoystickButton thisButton;
    private CommandDetails commandDetails;
    private ControllerCollection cc;

    private boolean lastState = false;

    public JoystickCommandPair(ControllerCollection cc, String commandInput, JoystickButton buttonToPair) {
        this.thisButton = buttonToPair;
        this.commandDetails = new CommandDetails(commandInput);
        this.cc = cc;
    }

    public void checkButton() {
        boolean currentState = this.thisButton.get();

        if (currentState && !this.lastState) {
            cc.callCommand(this.commandDetails);
        }

        if (!currentState && this.lastState && this.commandDetails.type() == CommandDetails.CommandType.SERIES) {
            cc.cancelCommand(this.commandDetails);
        }

        this.lastState = currentState;
    }
}