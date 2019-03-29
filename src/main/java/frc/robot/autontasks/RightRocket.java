package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class RightRocket extends AutonTask {
	/**
	 * Accepts the object of the running controlsProcessor to modify
	 *
	 * @param controlsProcessor
	 */
	public RightRocket(ControlsProcessor controlsProcessor) {
		super(controlsProcessor);

		queueTask("set_angular_offset -s -180");
		queueTask("hatch_intake -p");
		queueTask("hatch_true -p");

		queueTask("start_path -s");
		queueTask("delayed_to_position -p 5,100,1.5");

		queueTask("turn_to_angle_setpoint -s 290");

		queueTask("auton_vision_align -s 3.4");
		queueTask("upper_score -p");

		queueTask("extake -s");
		queueTask("add_backwards_line -p 6.8,21.14,3.8,24.14,5,5,0,0");
		queueTask("add_forwards_spline -p 6.8,21.14,300,1,5.5,5,270,6,10,12,0,0");

		queueTask("start_path -s");
		queueTask("delayed_to_position -p 3,110,2");

//		queueTask("turn_to_angle_setpoint -s 270")

		queueTask("hatch_station_intake -s");
		queueTask("auton_vision_align -p 1.8");  // Old: 3.5
//		queueTask("add_backwards_spline -p 6,0,270,4,7.75,15.35,240,4,10,12,0,0");
//
//		queueTask("start_path -s");
//		queueTask("flex_score -s");
//		queueTask("extake -s");
//		queueTask("add_backwards_spline -s 7.3,-4.5,268,2,8.3,10.5,240,3,10,8,0,0");
//		queueTask("start_path -s");
	}
}