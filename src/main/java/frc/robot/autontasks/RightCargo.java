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

		queueTask("auton_vision_align -s 4.25");
		queueTask("add_backwards_line -p 1.0,20,5,20,10,7,0,0");

		queueTask("set_current_position -s 1,20");

		queueTask("extake -s");

		queueTask("start_path -s");

		queueTask("turn_to_angle_setpoint -s 240");
		queueTask("add_forwards_spline -p 5,20,270,3,4,2.5,270,3,7,7,0,0");

		queueTask("set_current_position -s 5,20");

		queueTask("start_path -s");
		queueTask("floor_cargo_position -p");
		queueTask("cargo_intake -p");
	}
}
