/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

public class MotionPose {
    
    double angle, velocity, x, y;

    public MotionPose (double angle, double velocity, double x, double y) {
        this.angle = angle;
        this.velocity = velocity;
        this.x = x;
        this.y = y;
    }

    public double getOrthogonalDisplacement(double currentX, double currentY) {
        double errorDistance = distanceCalc(this.x, currentX, this.y, currentY);
        double errorY = currentY - this.y;
        double errorX = currentX - this.x;
        double errorAngle = 180 - Math.sinh(((Math.cos(this.angle) * errorX) + (Math.sin(this.angle) * errorY)) / errorDistance) / Math.PI * 180;

        return Math.sin(errorAngle) * errorDistance;
    }

    public double getTangentialDisplacement(double currentX, double currentY) {
        double errorDistance = distanceCalc(this.x, currentX, this.y, currentY);
        double errorY = currentY - this.y;
        double errorX = currentX - this.x;
        double errorAngle = 180 - Math.sinh(((Math.cos(this.angle) * errorX) + (Math.sin(this.angle) * errorY)) / errorDistance) / Math.PI * 180;

        return Math.cos(errorAngle) * errorDistance;
    }

    // Calculate distance
    public double distanceCalc(double x1, double x2, double y1, double y2) {
        return Math.pow((Math.pow(x2 - x1, 2)) + (Math.pow(y2 - y1, 2)), 0.5);
    }

}
