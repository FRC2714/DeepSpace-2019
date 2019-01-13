package frc.robot.autontasks;

import frc.robot.util.AutonTask;

public class testAuton extends AutonTask {

    public testAuton () {

        queueTask("add_forwards_spline -s 0,0,0,0,0,0,0,10,5,5,0,0");
        

    }

}
