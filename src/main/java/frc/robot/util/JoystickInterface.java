package frc.robot.util;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class JoystickInterface {

    private ArrayList<JoystickCommandPair> controls;
    private ControllerCollection cc_reference;

    public JoystickInterface(ControllerCollection cc_ref) {
        this.cc_reference = cc_ref;
    }

    public void append(String command, JoystickButton button) {
        controls.add(new JoystickCommandPair(this.cc_reference, command, button));
    }

    public void checkButtons() {
		controls.forEach((k) -> k.checkButton());
    }
}