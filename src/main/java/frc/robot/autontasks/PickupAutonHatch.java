package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class PickupAutonHatch extends AutonTask {

	public PickupAutonHatch(ControlsProcessor controlsProcessor) {
		super(controlsProcessor);


		queueTask("set_angular_offset -s -180");

//		queueTask("add_backwards_spline -s 0,0,270,2,0,8,270,2,5,5,0,0");
//		queueTask("add_backwards_spline -s 0,0,270,4,-5,8,270,2,5,5,0,0");

		queueTask("station_position -p");

		queueTask("add_backwards_spline -s 0,0,270,2,-4,12,270,4,10,5,0,0");
		queueTask("start_path -s");


		queueTask("add_forwards_spline -s -4,12,270,4,-6.75,-2.5,270,5,10,5,0,0");
		queueTask("start_path -s");

//
//		queueTask("hatch_station_intake -t 4");


		// queueTask("add_backwards_spline -s -6.75,-2.5,270,5,-6.75,3,270,3,10,5,0,0");
		// queueTask("start_path -t ");
	}
}