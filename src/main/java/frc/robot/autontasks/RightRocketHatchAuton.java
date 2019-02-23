package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class RightRocketHatchAuton extends AutonTask {

	public RightRocketHatchAuton(ControlsProcessor controlsProcessor) {
		super(controlsProcessor);


		queueTask("set_angular_offset -s -180");

		// queueTask("add_backwards_spline -s 0,0,270,6,-10,15,270,4,8,10,0,0");
		// queueTask("add_forwards_spline -s -10,15,270,4,0,0,270,6,8,10,0,0");

		// queueTask("add_backwards_spline -s 0,0,270,6,-10,15,270,4,8,10,0,0");
		// queueTask("add_forwards_spline -s -10,15,270,4,0,0,270,6,8,10,0,0");

		// queueTask("add_backwards_spline -s 0,0,270,6,-10,15,270,4,8,10,0,0");
		// queueTask("add_forwards_spline -s -10,15,270,4,0,0,270,6,8,10,0,0");

		// queueTask("add_backwards_spline -s 0,0,270,6,-10,15,270,4,8,10,0,0");
		// queueTask("add_forwards_spline -s -10,15,270,4,0,0,270,6,8,10,0,0");

		// queueTask("add_backwards_spline -s 0,0,270,6,-10,15,270,4,8,10,0,0");
		// queueTask("add_forwards_spline -s -10,15,270,4,0,0,270,6,8,10,0,0");

		queueTask("add_backwards_spline -s 0,0,270,4,3,21,310,9,12,10,0,0");
		queueTask("start_path -s");
		queueTask("auton_vision_align -s");
		queueTask("add_backwards_spline -s 7.2,16.5,298,1,4.5,19.5,270,1,10,8,0,0");
		queueTask("start_path -s");
		queueTask("add_forwards_spline -s 4,19.7,270,2,7.5,-3,268,4,12,10,0,0");
		queueTask("start_path -s");
		queueTask("auton_vision_align -s");
		queueTask("add_backwards_spline -s 7.3,-4.5,268,2,9,10.5,240,3,10,8,0,0");
		queueTask("start_path -s");



//		queueTask("add_backwards_spline -s 3,21,310,2,1,23,270,2,5,5,0,0");
//		queueTask("add_backwards_spline_dynamic -s 1,2,24,310,4,5,5,0,0");
//		queueTask("start_path -s");
//		queueTask("add_backwards_spline -s 3,21,310,4,,9.4,294,4,5,3,0,0");
//		queueTask("add_forwards_spline -s 6.6,9.4,294,2,7.6,-2,270,6,12,12,0,0");
//		queueTask("add_backwards_spline -s 7.6,-2.5,270,2,4,22,270,8,12,12,0,0");
//		queueTask("add_forwards_spline -s 4,22,270,1,-7.2,17,240,2,13,5,0,0");

		//Alisa says, "try only splines".
	}
}
