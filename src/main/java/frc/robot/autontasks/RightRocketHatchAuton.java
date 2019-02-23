package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class RightRocketHatchAuton extends AutonTask {

	public RightRocketHatchAuton(ControlsProcessor controlsProcessor) {
		super(controlsProcessor);



		queueTask("set_angular_offset -s -180");

		queueTask("add_backwards_spline -s 0,0,270,4,4.75,14.25,270,6,12,10,0,8");
		queueTask("add_backwards_spline -s 4.75,14.25,270,1,4.5,20.5,304,2,12,8,8,0");

		queueTask("start_path -s");

		queueTask("auton_vision_align -s");
		queueTask("add_backwards_spline -s 7.2,16.5,298,1,4.5,19.5,270,1,10,8,0,0");
		queueTask("start_path -s");
		queueTask("add_forwards_spline -s 4,19.7,270,2,7.5,-3,268,4,14,12,0,0");
		queueTask("start_path -s");
		queueTask("auton_vision_align -s");
		queueTask("add_backwards_spline -s 7.3,-4.5,268,2,9,10.5,240,3,10,8,0,0");
		queueTask("start_path -s");

//		queueTask("set_angular_offset -s -180");
//
//		queueTask("add_backwards_spline -s 0,0,270,7,4.75,18,270,7,12,10,0,8");
//		queueTask("add_backwards_spline -s 4.75,18,270,1,4.5,24.5,304,2,12,8,8,0");
//		queueTask("start_path -s");
//		queueTask("auton_vision_align -s");
//		queueTask("set_current_position -s 7.2,16.5");
//		queueTask("add_backwards_spline -s 7.2,16.5,298,1,4.5,19.5,270,1,10,8,0,0");
//		queueTask("start_path -s");
//		queueTask("add_forwards_spline -s 4,19.7,270,2,7.5,-3,268,4,14,12,0,0");
//		queueTask("start_path -s");
//		queueTask("auton_vision_align -s");
//		queueTask("add_backwards_spline -s 7.3,-4.5,268,2,9,10.5,240,3,10,8,0,0");
//		queueTask("start_path -s");



		//Alisa says, "try only splines".
	}
}
