package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class TestTask extends AutonTask {

	public TestTask(ControlsProcessor controlsProcessor) {
		super(controlsProcessor);

		queueTask("set_angular_offset -s 180");
		queueTask("add_backwards_spline -s 0,0,270,2,0,5,270,2,5,5,0,0");
		queueTask("start_path -s");
		queueTask("add_backwards_spline_dynamic -s 2,0,10,270,2,5,5,0,0");

//		queueTask("add_backwards_spline_dynamic -s 2,5,10,270,2,5,5,0,0");
		queueTask("start_path -s");

	}
}
