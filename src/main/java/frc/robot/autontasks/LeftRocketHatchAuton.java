package frc.robot.autontasks;

import frc.robot.util.AutonTask;
import frc.robot.util.ControlsProcessor;

public class LeftRocketHatchAuton extends AutonTask {

    public LeftRocketHatchAuton(ControlsProcessor controlsProcessor) {
        super(controlsProcessor);

        queueTask("set_angular_offset -s -180");

        queueTask("add_backwards_spline -s 0,0,270,4,-4.75,14.25,270,6,12,10,0,8");
        queueTask("add_backwards_spline -s -4.75,14.25,270,1,-4.5,20.5,236,2,12,8,8,0");

        queueTask("start_path -s");

        queueTask("auton_vision_align -s");
        queueTask("add_backwards_spline -s -7.2,16.5,242,1,-4.5,19.5,270,1,10,8,0,0");
        queueTask("start_path -s");
        queueTask("add_forwards_spline -s -4.25,19.7,270,2,-7,-3,268,4,14,12,0,0");
        queueTask("start_path -s");
        queueTask("auton_vision_align -s");
        queueTask("add_backwards_spline -s -7.3,-4.5,268,2,-8.25,10.5,296,3,10,8,0,0");
        queueTask("start_path -s");
    }














}