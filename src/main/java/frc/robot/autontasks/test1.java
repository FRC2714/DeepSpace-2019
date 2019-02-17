package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class test1 extends AutonTask {

	public test1(ControlsProcessor controlsProcessor) {
		super(controlsProcessor);


		queueTask("set_angular_offset -s -180");

		queueTask("add_backwards_spline -s 0,0,270,2,-5.3,6,0,2,12,5,0,5");
		queueTask("add_backwards_spline -s -5.3,6,0,2,-10.6,0,90,2,12,5,5,0");
		queueTask("add_forwards_spline -s -10.6,0,90,2,-5.3,6,0,2,12,5,0,5");
		queueTask("add_forwards_spline -s -5.3,6,0,2,0,0,270,2,12,5,5,0");


		queueTask("start_path -s");
	}
}