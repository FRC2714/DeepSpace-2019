package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class RightCargoHatchAuton extends AutonTask {

	public RightCargoHatchAuton(ControlsProcessor controlsProcessor) {
		super(controlsProcessor);

		// one spline
		// queueTask("add_forwards_spline -s 0,0,90,6,3.75,14,90,4,10,8,0,5");
		// queueTask("add_forwards_spline -s 3.75,14,90,2,0.5,17.1,180,3,5,5,5,0");
		queueTask("set_angular_offset -s -180");
        queueTask("add_backwards_spline -s 0,0,270,3,3.75,20.5,220,5,5,5,0,0");
        queueTask("add_forwards_spline -s 3.75,20.5,220,2,0.5,18,180,2.5,5,4.5,0,0");
		queueTask("start_path -s");
		// queueTask("auton_vision_align -s 4.9");
		// queueTask("add_backwards_spline -s 0.1,17.1,175,0.5,3.5,20.5,270,3,5,5,0,0");
		// queueTask("start_path -s");
		// // queueTask("add_forwards_spline -s 3.5,20.5,270,5,6,-3,270,5,5,5,0,0");
		// queueTask("start_path -s");
		// queueTask("auton_vision_align -s 4.5");
		// queueTask("add_forwards_spline -s 6.1,-4.7,261,");


	}
}
