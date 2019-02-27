package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class RightRocketHatchAuton extends AutonTask {

	public RightRocketHatchAuton(ControlsProcessor controlsProcessor) {
		super(controlsProcessor);



		queueTask("set_angular_offset -s -180");

		queueTask("hatch_intake -p");
		queueTask("hatch_true -p");
		queueTask("add_backwards_spline -p 0,0,270,4,3.75,14.25,270,6,12,10,0,8");
		queueTask("add_backwards_spline -p 3.75,14.25,270,1,3.5,20.5,304,2,8,8,8,0");
		queueTask("start_path -s");
		queueTask("upper_score -s");
		queueTask("auton_vision_align -s 5");
		queueTask("extake -s");

		queueTask("add_backwards_spline -s 6.2,16.5,298,1,3.5,19.5,270,1,5,5,0,0");
		queueTask("start_path -s");
		queueTask("station_position -s");
		queueTask("add_forwards_spline -s 3.5,19.7,270,2,6.7,-2,268,6.5,10,8,0,0");
		queueTask("start_path -s");
		queueTask("auton_vision_align -s 4.7");


//		queueTask("hatch_station_intake -s");
//		queueTask("add_backwards_spline -s 7,-4.5,268,2,8.3,10.5,240,3,10,8,0,0");
//		queueTask("start_path -s");
//		queueTask("back_score -s");
//		queueTast("extake -s");


	}
}
