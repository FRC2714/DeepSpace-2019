package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class LeftCargoHabTwoAuton extends AutonTask {
	/**
	 * Accepts the object of the running controlsProcessor to modify
	 *
	 * @param controlsProcessor
	 */
	public LeftCargoHabTwoAuton(ControlsProcessor controlsProcessor) {
		super(controlsProcessor);

		queueTask("hatch_intake -p");
		queueTask("hatch_true -p");

		// Start position to far cargo bay
		queueTask("start_position -s");
		queueTask("add_forwards_spline -s 0,0,90,10,-3.25,23,15,6,4,12,0,3");
		queueTask("start_path -s");
		queueTask("station_position -s");
		queueTask("auton_vision_align -p 4.2");
		queueTask("extake -s");;


		// Backwards spline from far cargo bay
		queueTask("add_backwards_spline -p 0.55,21.3,0,2,-4.45,21,315,2,5,12,0,0");
		queueTask("start_path -s");

		// Backwards spline to cargo
		queueTask("floor_cargo_position -s");
		queueTask("add_forwards_spline -p -4.45,19.3,315,4,-2,1,270,10,4,12,0,0");
		queueTask("start_path -p");
		queueTask("cargo_intake -p");
	}
}
