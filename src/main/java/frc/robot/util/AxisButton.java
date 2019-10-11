package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class AxisButton extends JoystickButton {

	private Joystick joystick;
	private int axis;
	public AxisButton(Joystick joystick, int axis){
		super(joystick, 20);
		this.joystick = joystick;
	}

	@Override
	public boolean get() {
		System.out.println("True? " + joystick.getRawAxis(axis) + " " + (joystick.getRawAxis(axis) < -0.5));
		return joystick.getRawAxis(axis) < -0.5;
	}
}
