package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class RightCargoHatchAuton extends AutonTask {

	public RightCargoHatchAuton(ControlsProcessor controlsProcessor) {
		super(controlsProcessor);

		queueTask("add_forwards_spline -s 0,0,90,5,5,12,90,4,10,8,0,8");
		queueTask("add_forwards_spline -s 5,12,90,2,0.5,17.1,180,3,10,8,8,0");
		queueTask("start_path -s");
		queueTask("auton_vision_align -s 7.1");
		queueTask("add_backwards_spline -s 0.1,17.1,175,0.5,3.5,20.5,270,3,5,5,0,0");
		queueTask("start_path -s");
		queueTask("add_forwards_spline -s 3.5,20.5,270,5,6,-3,270,5,10,8,0,0");
		queueTask("start_path -s");
		queueTask("auton_vision_align -s 6.4");
		queueTask("add_forwards_spline -s 6.1,-4.7,261,");


	}
}
