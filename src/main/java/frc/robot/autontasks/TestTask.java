package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class TestTask extends AutonTask {
	public TestTask(ControlsProcessor controlsProcessor) {
		super(controlsProcessor);
		queueTask("add_forwards_spline -s 0,0,90,3,5,11,90,3,7,7,0,4");
		queueTask("start_endless_path -s");
		queueTask("delayed_to_position -p 0.6,86,1.0");
		queueTask("hatch_station_intake -p");
		queueTask("spline_auton_vision_align -p 2.9");
	}
}