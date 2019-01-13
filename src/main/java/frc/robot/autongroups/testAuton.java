package frc.robot.autongroups;

import frc.robot.util.AutoTaskGroup;

public class testAuton extends AutoTaskGroup {

    @Override
    public void registerTasks() {
        addToQueue("add_forwards_spline -s 0,0,0,0,0,0,4,4,8,3,0,0");
    }

}

