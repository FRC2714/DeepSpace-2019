package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class RightCargoHatchAuton extends AutonTask {
	/**
	 * Accepts the object of the running controlsProcessor to modify
	 *
	 * @param controlsProcessor
	 */
	public RightCargoHatchAuton(ControlsProcessor controlsProcessor) {
		super(controlsProcessor);

		queueTask("add_forwards_spline -s 0,0,0,0,0,0,0,3,13,4,0,4");
		queueTask("add_forwards_spline -s 0,0,5,5,3,6,8,11,8,8,4,5");
		queueTask("add_forwards_spline -s 5,5,3,-0.7,11,14,16.9,16.9,5,5,5,0");
		queueTask("add_backwards_spline -s -0.7,3,5,5,16.9,16.9,20,22,5,5,0,0");
		queueTask("start_path -s");

	}
}
