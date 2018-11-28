package frc.robot.util;

import java.util.ArrayList;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;

public class SplineFactory {
  private double x1, x2, x3, x4, y1, y2, y3, y4, acceleration, velocity;
  private ArrayList<Double> xValues = new ArrayList<Double>();
  private ArrayList<Double> yValues = new ArrayList<Double>();
  int valuesPushed = 0;

  private double tolerance = 0.00001;
  private double tStep = 0.001;
  private double period = 0.000050;

  private FileWriter fileWriter = null;

  public SplineFactory(double x1, double x2, double x3, double x4, double y1, double y2, double y3, double y4,
      double acceleration, double velocity) {

    this.x1 = x1;
    this.x2 = x2;
    this.x3 = x3;
    this.x4 = x4;
    this.y1 = y1;
    this.y2 = y2;
    this.y3 = y3;
    this.y4 = y4;

    this.acceleration = acceleration;
    this.velocity = velocity;

    try {
      this.fileWriter = new FileWriter("./output.csv");
    } catch (IOException e) {
      e.printStackTrace();
    }

    //Fill point buffer

    double backT = 1;
    double frontT = 0;
    int placement = 0;
    double currentVelocity = 0;

    while (frontT <= backT) {
      //Find front and back position
      frontT = binaryFind(frontT, currentVelocity * this.period, placement);
      backT = binaryFind(backT, -currentVelocity * this.period, placement);
      placement++;

      //Recalculate velocity with acceleration
      if (currentVelocity < this.velocity) {
        currentVelocity += this.acceleration * this.period;
      }
    }

    //Print out into a point buffer
    try {
      for (double point : this.xValues) {
        this.fileWriter.append(Double.toString(point) + "\n");
      }
      this.fileWriter.flush();
      this.fileWriter.close();
    } catch (IOException e) {
      e.printStackTrace();
    }
    
  }

  double binaryFind(double startT, double distance, int location){
    double internalT = startT;
    double tStep_modified = this.tStep;

    double newX = quarticCalc(internalT, this.x1, this.x2, this.x3, this.x4);
    double newY = quarticCalc(internalT, this.y1, this.y2, this.y3, this.y4);
    double startX = newX;
    double startY = newY;

    double distanceAway = 0;

    boolean direction;
    boolean lastDirection;

    if ((distance - distanceAway) < 0){
      lastDirection = true;
    } else {
      lastDirection = false;
    }

    do {
      if ((distance - distanceAway) < 0){
        internalT -= tStep_modified;
        direction = false;
      } else {
        internalT += tStep_modified;
        direction = true;
      }

      if(direction != lastDirection){
        tStep_modified /= 2;
      }

      lastDirection = direction;

      newX = quarticCalc(internalT, this.x1, this.x2, this.x3, this.x4);
      newY = quarticCalc(internalT, this.y1, this.y2, this.y3, this.y4);

      distanceAway = distanceCalc(newX, startX, newY, startY);

    } while (Math.abs(distance - distanceAway) < this.tolerance);

    xValues.add(location, newX);
    yValues.add(location, newY);

    return internalT;
  }

  // Calculate quartic spline point with 4 controls
  public double quarticCalc(double t, double c1, double c2, double c3, double c4) {
    return (Math.pow(1 - t, 3) * c1) + 3 * (Math.pow(1 - t, 2) * t * c2) + 3 * (Math.pow(t, 2) * (1 - t) * c3)
        + (Math.pow(t, 3) * c4);
  }

  // Calculate distance
  public double distanceCalc(double x1, double x2, double y1, double y2) {
    return Math.pow((Math.pow(x2-x1, 2)) + (Math.pow(y2-y1, 2)),0.5);
  }

}