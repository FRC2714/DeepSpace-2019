package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class TestAuton extends AutonTask {

    public TestAuton(ControlsProcessor controlsProcessor) {
        super(controlsProcessor);
        
        // queueTask("add_forwards_spline -s 0,0,0,0,0,0,0,10,5,5,0,0");
        queueTask("set_angular_offset -s -180");
        queueTask("add_backwards_spline -s 0,0,5.95,7.25,0,7,7.95,10.45,13,13,0,0");
        queueTask("add_forwards_spline -s 7.25,5.95,7,7,10.45,7.95,0,0,13,13,0,0");
        queueTask("start_path -s");
    }
}