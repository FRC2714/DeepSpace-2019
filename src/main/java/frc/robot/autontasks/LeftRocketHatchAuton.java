package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class LeftRocketHatchAuton extends AutonTask {

    public LeftRocketHatchAuton(ControlsProcessor controlsProcessor) {
        super(controlsProcessor);
        

            queueTask("set_angular_offset -s -180");
            queueTask("add_backwards_spline -s 0,0,270,9,-6.4,9.4,294,4,8,10,0,0");
            queueTask("add_forwards_spline -s -6.4,9.4,294,2,-7,-3.75,270,5,12,12,0,0");
            queueTask("add_backwards_spline -s -7,-3.75,270,5,-4.5,20,270,5,12,12,0,0");
            
            //adjust final points
            queueTask("add_forwards_spline -s -4.5,20,270,2,-6.4,15.35,240,2,13,5,0,0");
//         queueTask("add_backwards_spline -s 5.2,5.2,5.2,4.3,13.5,15,18,19.5,13,8,8,0");
//         queueTask("add_forwards_spline -s 4.8,4.8,4.8,6.75,19.5,19.5,19.5,17.5,13,8,0,0");
            queueTask("start_path -s");
    }
}