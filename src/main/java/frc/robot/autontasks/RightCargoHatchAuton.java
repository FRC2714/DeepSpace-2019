package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class RightCargoHatchAuton extends AutonTask {

	public RightCargoHatchAuton(ControlsProcessor controlsProcessor) {
		super(controlsProcessor);
		queueTask("set_angular_offset -s -180");
		queueTask("add_backwards_spline -s 0,0,270,3,5,20.5,220,5,12,8,0,0");
		queueTask("add_forwards_spline -s 5,20.5,220,2,-0.5,16,180,2.5,12,4.5,0,0");
//		queueTask("add_backwards_spline -s 0.5,18,0,2.5,-5,20.5,320,2,12,4.5,0,0");
//		queueTask("add_forwards_spline -s -5,20.5,320,5,-6.5,-2.5,270,5,12,12,0,0");
//		queueTask("add_backwards_spline -s -6.5,-2.5,270,5,-5,20.5,340,3,12,12,0,0");
//		queueTask("add_forwards_spline -s -5,20.5,340,0.5,0.5,18.85,0,1.5,12,4.5,0,0");


		//queueTask("add_forwards_spline -s 0,0,90,2,-4,9.5,60,2,8,10,0,3");
		//queueTask("add_forwards_spline -s -4,9.5,60,2,0.5,17,0,3,12,3,3,0");
		//queueTask("add_backwards_spline -s 0.5,17,0,2,-3,19,270,2,12,4,4,0");
		//queueTask("add_forwards_spline -s -3,19,270,3,-3,23,270,4,12,4,4,0");
		queueTask("start_path -s");
	}
}
