package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class TestTask extends AutonTask {
	public TestTask(ControlsProcessor controlsProcessor) {
		super(controlsProcessor);

		queueTask("hatch_intake -p");
		queueTask("hatch_true -p");

//		queueTask("add_forwards_spline -s 0,0,90,3,5,11,90,3,7,7,0,4");
//		queueTask("start_endless_path -s");
//		queueTask("delayed_to_position -p 0.6,86,1.0");
//		queueTask("hatch_station_intake -p");
//		queueTask("spline_auton_vision_align -p 2.9");


		/*queueTask("set_angular_offset -s -180");
		queueTask("start_path -s");
		queueTask("delayed_to_position -p 0.6,86,1.5");

		queueTask("auton_vision_align -s 3.7");
		queueTask("add_backwards_line -p 7,17.5,4.2,21.4,10,10,0,0");

		queueTask("extake -s");

		queueTask("set_current_position -s 7,17.5");

		queueTask("start_path -s");
		*/

		queueTask("start_endless_path -s");
		queueTask("spline_auton_vision_align -p 3.45");
		queueTask("delayed_to_position -p 0.6,86,1.0");

		queueTask("extake -s");
		queueTask("add_backwards_spline -p 6.8,14.5,67,3,6,2.5,90,4,10,10,0,0");
//
		queueTask("set_current_position -s 6.8,14.5");
//
		queueTask("start_path -s");

	}
}