package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.JoystickCommandPair;

public class OI {

	// Controllers and button boxes
	Joystick xbox1 = new Joystick(RobotMap.p_xbox1);
	Joystick newButtonBoxA = new Joystick(RobotMap.p_newButtonBoxA);
	Joystick newButtonBoxB = new Joystick(RobotMap.p_newButtonBoxB);
	Joystick buttonBoxA = new Joystick(RobotMap.p_buttonBox);

	// xbox1 buttons
	JoystickButton a = new JoystickButton(xbox1, 1);
	JoystickButton b = new JoystickButton(xbox1, 2);
	JoystickButton x = new JoystickButton(xbox1, 3);
	JoystickButton y = new JoystickButton(xbox1, 4);
	JoystickButton lb = new JoystickButton(xbox1, 5);
	JoystickButton rb = new JoystickButton(xbox1, 6);
	JoystickButton back = new JoystickButton(xbox1, 7);
	JoystickButton start = new JoystickButton(xbox1, 8);
	

	// buttonBoxA buttons
	JoystickButton bb11 = new JoystickButton(buttonBoxA, 16);
	JoystickButton bb12 = new JoystickButton(buttonBoxA, 10);
	JoystickButton bb13 = new JoystickButton(buttonBoxA, 15);
	JoystickButton bb14 = new JoystickButton(buttonBoxA, 13);
	JoystickButton bb15 = new JoystickButton(buttonBoxA, 14);
	JoystickButton bb16 = new JoystickButton(buttonBoxA, 5);
	JoystickButton bb17 = new JoystickButton(buttonBoxA, 11);
	JoystickButton bb18 = new JoystickButton(buttonBoxA, 12);
	JoystickButton bb19 = new JoystickButton(buttonBoxA, 3);
	JoystickButton bb110 = new JoystickButton(buttonBoxA, 4);
	JoystickButton bb111 = new JoystickButton(buttonBoxA, 8);
	JoystickButton bb112 = new JoystickButton(buttonBoxA, 7);
	JoystickButton bb113 = new JoystickButton(buttonBoxA, 9);

	// newButtonBoxA

	JoystickButton nbba1 = new JoystickButton(newButtonBoxA, 1);
	JoystickButton nbba2 = new JoystickButton(newButtonBoxA, 2);
	JoystickButton nbba3 = new JoystickButton(newButtonBoxA, 3);
	JoystickButton nbba4 = new JoystickButton(newButtonBoxA, 4);
	JoystickButton nbba5 = new JoystickButton(newButtonBoxA, 5);
	JoystickButton nbba6 = new JoystickButton(newButtonBoxA, 12);
	JoystickButton nbba7 = new JoystickButton(newButtonBoxA, 11);
	JoystickButton nbba8 = new JoystickButton(newButtonBoxA, 6);
	JoystickButton nbba9 = new JoystickButton(newButtonBoxA, 7);
	JoystickButton nbba10 = new JoystickButton(newButtonBoxA, 8);
	JoystickButton nbba11 = new JoystickButton(newButtonBoxA, 9);
	JoystickButton nbba12 = new JoystickButton(newButtonBoxA, 10);

	// newButtonBoxB
	JoystickButton nbbb1 = new JoystickButton(newButtonBoxB, 1);
	// JoystickButton nbbb2 = new JoystickButton(newButtonBoxB, 2);
	JoystickButton nbbb3 = new JoystickButton(newButtonBoxB, 3);
	JoystickButton nbbb4 = new JoystickButton(newButtonBoxB, 11);
	JoystickButton nbbb5 = new JoystickButton(newButtonBoxB, 10);
	JoystickButton nbbb6 = new JoystickButton(newButtonBoxB, 9);
	JoystickButton nbbb7 = new JoystickButton(newButtonBoxB, 8);
	JoystickButton nbbb8 = new JoystickButton(newButtonBoxB, 7);
	JoystickButton nbbb9 = new JoystickButton(newButtonBoxB, 6);
	JoystickButton nbbb10 = new JoystickButton(newButtonBoxB, 5);
	JoystickButton nbbb11 = new JoystickButton(newButtonBoxB, 4);
	JoystickButton nbbb12 = new JoystickButton(newButtonBoxB, 3);
	JoystickButton nbbb13 = new JoystickButton(newButtonBoxB, 2);
	JoystickButton nbbb14 = new JoystickButton(newButtonBoxB, 1);
	public ArrayList<JoystickCommandPair> buttons = new ArrayList<JoystickCommandPair>();

	public OI() {
		this.buttons.add(new JoystickCommandPair("intake_in -s", this.a));
		this.buttons.add(new JoystickCommandPair("set_angular_offset -s -90", this.start));
		this.buttons.add(new JoystickCommandPair("add_forwards_spline -s 0,0,0,0,0,0,0,10,5,5,0,0", this.y));
		this.buttons.add(new JoystickCommandPair("add_forwards_spline -s 4.5,8.5,6,9,0,0,4,11,10,3,3,0", this.x));
		//this.buttons.add(new JoystickCommandPair("target_point -s 1,1", this.y));
		this.buttons.add(new JoystickCommandPair("add_forwards_spline -s 0,0,0,0,0,0,4,4,8,3,0,0", this.y));
		this.buttons.add(new JoystickCommandPair("add_forwards_spline -s 0,0,5.5,8.75,5,7,1.5,9.5,8,3,0,0", this.x));
		this.buttons.add(new JoystickCommandPair("add_backwards_spline -s 8.75,5.5,8,8,8,1.5,0,-5,8,3,0,0", this.a));
		this.buttons.add(new JoystickCommandPair("start_path -s", this.b));

	}

	public double getLeftJoystick() {
		return xbox1.getRawAxis(1);
	}

	public double getRightJoystick() {
		return xbox1.getRawAxis(4);
	}
	

}