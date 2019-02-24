package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class RightCargoLevelTwoAuton extends AutonTask {
	/**
	 * Accepts the object of the running controlsProcessor to modify
	 *
	 * @param controlsProcessor
	 */
	public RightCargoLevelTwoAuton(ControlsProcessor controlsProcessor) {
		super(controlsProcessor);

		queueTask("set_angular_offset -s -180");
		queueTask("add_backwards_spline -s 0,0,270,3,5,20.5,220,5,12,12,0,0");
		queueTask("add_forwards_spline -s -5,20.5,320,2,0.5,18,0,2.5,12,4.5,0,0");
//		queueTask("add_backwards_spline -s 0.5,18,0,2.5,-5,20.5,320,2,12,4.5,0,0");
//		queueTask("add_forwards_spline -s -5,20.5,320,5,-6.5,-2.5,270,5,12,12,0,0");
//		queueTask("add_backwards_spline -s -6.5,-2.5,270,5,-5,20.5,340,3,12,12,0,0");
//		queueTask("add_forwards_spline -s -5,20.5,340,0.5,0.5,18.85,0,1.5,12,4.5,0,0");
		queueTask("start_path -s");
	}
}
