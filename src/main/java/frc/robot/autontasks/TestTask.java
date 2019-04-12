package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class TestTask extends AutonTask {
	public TestTask(ControlsProcessor controlsProcessor) {
		super(controlsProcessor);

		queueTask("turn_to_angle_setpoint -s 180");
		queueTask("turn_to_angle_setpoint -s 90");
	}
}