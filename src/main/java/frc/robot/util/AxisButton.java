package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class AxisButton extends JoystickButton {

	private Joystick joystick;
	private int axis;
	boolean isNegative;

	public AxisButton(Joystick joystick, int axis, boolean isNegative){
		super(joystick, 20);
		this.joystick = joystick;
		this.isNegative = isNegative;
	}

	@Override
	public boolean get() {
		if(isNegative) {
			return joystick.getRawAxis(axis) < -0.5;
		}
		else{
			return joystick.getRawAxis(axis) > 0.5;
		}
	}
}
