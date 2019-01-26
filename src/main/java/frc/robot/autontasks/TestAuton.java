package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class TestAuton extends AutonTask {

    public TestAuton(ControlsProcessor controlsProcessor) {
        super(controlsProcessor);
        queueTask("add_forwards_spline -s 0,0,0,0,0,0,0,10,5,5,0,0");
        queueTask("start_path -s");
    }
}