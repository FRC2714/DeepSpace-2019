package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class LeftCargoHatchHabTwo extends AutonTask {

    public LeftCargoHatchHabTwo(ControlsProcessor controlsProcessor) {
        super(controlsProcessor);

        queueTask("add_forwards_spline -s 0,0,90,2,-1,22.55,180,2,5,5,0,0");
        queueTask("start_path -s");
        queueTask("auton_vision_align -s 4.05");

        // queueTask("add_backwards_spline -s 0,22.55,180,2,-6,20,180,2,5,5,0,0");
        // queueTask("add_forwards_spline -s -6,20,180,2,-7.3,1.3,270,2,5,5,0,0");
        // queueTask("start_path -s");
        // queueTask("auton_vision_align -s 5.3");

        // queueTask("add_backwards_spline -s -7.3,-1.1,270,2,-4.8,20.7,180,2,5,5,0,0");
        // queueTask("add_forwards_spline -s -4.8,20.7,180,2,-1.1,20.7,180,2,5,5,0,0");
        // queueTask("start_path -s");
        // queueTask("auton_vision_align -s 4.05");
    }
}
            
