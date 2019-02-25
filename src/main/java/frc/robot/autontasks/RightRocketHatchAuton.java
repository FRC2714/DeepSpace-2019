package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class RightRocketHatchAuton extends AutonTask {

	public RightRocketHatchAuton(ControlsProcessor controlsProcessor) {
		super(controlsProcessor);



		queueTask("set_angular_offset -s -180");

		queueTask("hatch_intake -p");
		queueTask("hatch_true -p");
		// queueTask("station_position -p");
		queueTask("add_backwards_spline -p 0,0,270,4,4.75,14.25,270,6,12,10,0,8");
		queueTask("add_backwards_spline -p 4.75,14.25,270,1,4.5,20.5,304,2,10,8,8,0");
		queueTask("start_path -s");
		queueTask("upper_score -s");
		queueTask("wait -p 1");
		queueTask("auton_vision_align -p 6.3");
		queueTask("extake -s");
//		queueTask("hatch_station_position -p");
//		queueTask("add_backwards_spline -s 7.2,16.5,298,1,4.5,19.5,270,1,10,8,0,0");
//		queueTask("start_path -s");
//		queueTask("add_forwards_spline -s 4,19.7,270,2,7,-3,268,4,14,12,0,0");
//		queueTask("start_path -s");
//		queueTask("auton_vision_align -p 6.3");
//		queueTask("hatch_station_intake -s");
//		queueTask("add_backwards_spline -s 7,-4.5,268,2,8.3,10.5,240,3,10,8,0,0");
//		queueTask("start_path -s");
//		queueTask("back_score -s");
//		queueTast("extake -s");


	}
}
