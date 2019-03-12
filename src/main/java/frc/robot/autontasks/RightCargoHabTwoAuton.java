package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class RightCargoHabTwoAuton extends AutonTask {
	/**
	 * Accepts the object of the running controlsProcessor to modify
	 *
	 * @param controlsProcessor
	 */
	public RightCargoHabTwoAuton(ControlsProcessor controlsProcessor) {
		super(controlsProcessor);

        queueTask("hatch_intake -p");
        queueTask("hatch_true -p");
        
        // Start position to far cargo bay
        queueTask("start_position -s");
        queueTask("add_forwards_spline -s 0,0,90,10,3.25,21.3,180,6,3,12,0,3");
        queueTask("start_path -s");
        queueTask("station_position -s");
        queueTask("auton_vision_align -p 4.2");
		queueTask("extake -s");
        
        // Backwards spline from far cargo bay
		queueTask("add_backwards_spline -p -0.55,21.3,180,2,4.45,19.3,225,2,3,12,0,0");
		queueTask("start_path -s");

		// Backwards spline to cargo
        queueTask("add_forwards_spline -p 4.45,19.3,225,4,1.45,1,270,10,3,12,0,0");
		queueTask("start_path -s");
		queueTask("floor_cargo_position -p");
		queueTask("cargo_intake -p");
	}
}
