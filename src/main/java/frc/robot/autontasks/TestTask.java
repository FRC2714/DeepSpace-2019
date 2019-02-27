package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class TestTask extends AutonTask {

	public TestTask(ControlsProcessor controlsProcessor) {
		super(controlsProcessor);

//		queueTask("set_angular_offset -s 180");
		queueTask("add_forwards_spline -s 0,0,90,1,-5,5,180,3,5,5,0,0");
//		queueTask("add_backwards_spline -s 0,0,270,2,0,5,270,2,5,5,0,0");
		queueTask("start_path -s");
//		queueTask("start_path -s");
//		queueTask("add_backwards_spline_dynamic -s 2,0,10,270,2,5,5,0,0");
//
////		queueTask("add_backwards_spline_dynamic -s 2,5,10,270,2,5,5,0,0");
//		queueTask("start_path -s");

//		queueTask("set_current_position -s 7.2,16.5");


//		queueTask("start_path -s");


	}
}
