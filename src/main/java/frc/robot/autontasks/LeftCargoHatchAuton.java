package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class LeftCargoHatchAuton extends AutonTask {

    public LeftCargoHatchAuton(ControlsProcessor controlsProcessor) {
        super(controlsProcessor);
        
        //Two splines
        // queueTask("set_angular_offset -s -180");
        // queueTask("add_backwards_spline -s 0,0,270,3,-5,21,320,5,12,12,0,0");
        // queueTask("add_forwards_spline -s -5,20.5,320,2,-1,18,0,2.5,12,4.5,0,0");
        queueTask("hatch_intake -p");
        queueTask("hatch_true -p");
        queueTask("add_forwards_spline -s 0,0,90,6,-3.75,13.5,90,4,12,10,0,5");
		queueTask("add_forwards_spline -s -3.75,13.5,90,2,0.5,17.1,0,3,5,5,5,0");
        queueTask("start_path -s");
//        queueTask("station_position -s");
        queueTask("go_to_position -s 14,90");
        queueTask("auton_vision_align -s 4.05");
        queueTask("extake -s");

        queueTask("add_backwards_spline -s 0.5,17.1,0,2,-3.5,20.5,270,3,5,5,0,0");
        queueTask("add_forwards_spline -s -3.5,20.5,270,6,-6,-3,270,4,10,8,0,0");
        queueTask("start_path -s");
        queueTask("hatch_station_intake -s");
        queueTask("auton_vision_align -s 5.3");
        queueTask("add_backwards_spline -s -6.1,-4.27,270,8,-5,21,320,5,12,10,0,0");
        queueTask("start_path -s");
        queueTask("add_forwards_spline -s -5,21,320,2,-1,18.5,0,2.5,5,5,0,0");
        queueTask("start_path -s");
        queueTask("auton_vision_align -s 5.3");

        
        // queueTask("add_forwards_spline -s -5,20.5,320,5,-6.5,-2.5,270,5,12,12,0,0");
        // queueTask("add_backwards_spline -s -6.5,-2.5,270,5,-5,20.5,340,3,12,12,0,0");
        // queueTask("add_forwards_spline -s -5,20.5,340,0.5,0.5,18.85,0,1.5,12,4.5,0,0");
            //queueTask("add_forwards_spline -s 0,0,90,2,-4,9.5,60,2,8,10,0,3");
            //queueTask("add_forwards_spline -s -4,9.5,60,2,0.5,17,0,3,12,3,3,0");
            //queueTask("add_backwards_spline -s 0.5,17,0,2,-3,19,270,2,12,4,4,0");
            //queueTask("add_forwards_spline -s -3,19,270,3,-3,23,270,4,12,4,4,0");
    }
}
            
