package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class TestTask extends AutonTask {


	public TestTask(ControlsProcessor controlsProcessor) {
		super(controlsProcessor);

		queueTask("add_forwards_spline -s 0,0,90,2,0,12,90,2,10,5,0,0");

		queueTask("start_vision_path -s 1000000,2");
	}
}
