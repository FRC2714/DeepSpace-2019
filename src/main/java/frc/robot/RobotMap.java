package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	// Controllers
	public static int p_xbox1 = 0;
	public static int p_buttonBox = 3;
	public static int p_newButtonBoxA = 1;
	public static int p_newButtonBoxB = 2;

	// DriveTrain Motors
	public static int p_leftDrive1 = 0;
	public static int p_leftDrive2 = 1;
	public static int p_rightDrive1 = 2;
	public static int p_rightDrive2 = 3;

	public static int p_driveShifter1 = 0;
	public static int p_driveShifter2 = 1;

	public static int p_leftEncoderA = 0;
	public static int p_leftEncoderB = 1;
	public static int p_rightEncoderA = 2;
	public static int p_rightEncoderB = 3;
}