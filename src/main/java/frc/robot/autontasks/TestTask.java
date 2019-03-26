package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class TestTask extends AutonTask {


	public TestTask(ControlsProcessor controlsProcessor) {
		super(controlsProcessor);

//		queueTask("add_forwards_spline -s 0,0,90,2,0,10,90,2,5,10,0,0"); //Simple
		queueTask("set_current_position -s 2,16");
		queueTask("add_forwards_spline -s 2,0,90,5,5,14,90,5,10,12,0,0"); //Simple S Curve
//		queueTask("add_forwards_spline -s 0,0,90,2,-3.182,13.567,180.03699761861517,2,5,5,0,0"); //Simple L curve
//		queueTask("add_forwards_spline -s 0,0,90,2,-19.279,0.012,264.5935014165078,2,5,5,0,0"); Start next to hab 1, end up in opposite loading station area
		queueTask("start_path -s");

	}
}
