package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class TestTask extends AutonTask {


	public TestTask(ControlsProcessor controlsProcessor) {
		super(controlsProcessor);

		// queueTask("add_forwards_spline -s 0,0,90,2,0,8,90,2,10,5,0,0");
		// queueTask("add_forwards_spline -s 0,0,90,3,3,8,90,3,10,5,0,0");
		queueTask("start_path -s");

	}
}
