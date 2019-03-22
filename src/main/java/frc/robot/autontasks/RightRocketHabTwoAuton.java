package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class RightRocketHabTwoAuton extends AutonTask {
	/**
	 * Accepts the object of the running controlsProcessor to modify
	 *
	 * @param controlsProcessor
	 */
	public RightRocketHabTwoAuton(ControlsProcessor controlsProcessor) {
		super(controlsProcessor);

		queueTask("set_angular_offset -s -180");

		queueTask("hatch_intake -p");
		queueTask("hatch_true -p");
		// queueTask("add_backwards_spline -s 0,0,270,7,4.75,18,270,7,12,10,0,8");
		// queueTask("add_backwards_spline -s 4.75,18,270,1,4.5,24.5,304,2,12,8,8,0");
		queueTask("start_path -s");
		queueTask("upper_score -s");
		queueTask("auton_vision_align -s 4.9");
		queueTask("extake -s");

		queueTask("set_current_position -s 7.2,16.5");
		queueTask("add_backwards_spline -s 7.2,16.5,298,1,4.5,18.5,270,1,10,8,0,0");
		queueTask("start_path -s");
		queueTask("station_position -s");

//		queueTask("add_forwards_spline -s 4,19.7,270,2,7.1,-2,268,4,14,12,0,0");
//		queueTask("start_path -s");
//		queueTask("auton_vision_align -s");
//		queueTask("add_backwards_spline -s 7.3,-4.5,268,2,8.3,10.5,240,3,10,8,0,0");
//		queueTask("start_path -s");
	}
}
