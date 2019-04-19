package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class RightCargo extends AutonTask {
	/**
	 * Accepts the object of the running controlsProcessor to modify
	 *
	 * @param controlsProcessor
	 */
	public RightCargo(ControlsProcessor controlsProcessor) {
		super(controlsProcessor);

		queueTask("hatch_intake -p");
        queueTask("hatch_true -p");

		queueTask("set_angular_offset -s -180");

		queueTask("start_path -s");
		queueTask("delayed_to_position -p 0.6,86,1.5");

		queueTask("auton_vision_align -s 3.95");
		queueTask("add_backwards_line -p 1.0,20,5,20,5,5,0,0");

		queueTask("set_current_position -s 1,20");

		queueTask("extake -s");

		queueTask("start_path -s");

		queueTask("turn_to_angle_setpoint -s 270");
		queueTask("add_forwards_spline -s 5,20,270,3,7.6,4,270,3,7,12,0,4");

		queueTask("set_current_position -s 5,20");

		queueTask("start_endless_path -s");
		queueTask("hatch_station_intake -p");
		queueTask("spline_auton_vision_align -p 2.4");
	}
}
