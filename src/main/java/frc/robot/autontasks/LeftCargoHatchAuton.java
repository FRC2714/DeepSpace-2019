package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class LeftCargoHatchAuton extends AutonTask {

    public LeftCargoHatchAuton(ControlsProcessor controlsProcessor) {
        super(controlsProcessor);
        queueTask("set_angular_offset -s -180");
        queueTask("add_backwards_spline -s 0,0,270,3,-5,20.5,300,5,12,10,0,0");
        queueTask("add_forwards_spline -s -5,20.5,300,2,0.5,18,0,5,12,4,0,0");
        //queueTask("add_forwards_spline -s -5,20.5,300,2,0.5,18,0,5,12,3,0,0");


            //queueTask("add_forwards_spline -s 0,0,90,2,-4,9.5,60,2,8,10,0,3");
            //queueTask("add_forwards_spline -s -4,9.5,60,2,0.5,17,0,3,12,3,3,0");
            //queueTask("add_backwards_spline -s 0.5,17,0,2,-3,19,270,2,12,4,4,0");
            //queueTask("add_forwards_spline -s -3,19,270,3,-3,23,270,4,12,4,4,0");
            queueTask("start_path -s");
    }
}
            