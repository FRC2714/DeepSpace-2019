package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class DelayAutonTesterTask extends AutonTask {

	/**
	 * Accepts the object of the running controlsProcessor to modify
	 *
	 * @param controlsProcessor
	 */
	public DelayAutonTesterTask(ControlsProcessor controlsProcessor) {
		super(controlsProcessor);

		queueTask("delay_tester -t 5");
	}
}
