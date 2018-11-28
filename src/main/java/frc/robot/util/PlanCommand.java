package frc.robot.util;


import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class PlanCommand extends Command {
  
  private Command command;
  private double activationPoint;

  public PlanCommand(Command command, double activationPoint) {
    this.command = command;
    this.activationPoint = activationPoint;
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    System.out.println("WAITING: " + Robot.splinePercentage);
    if (Robot.splinePercentage >= activationPoint) {
      System.out.println("ACTIVATED");
      this.command.start();
    }
  }
 
  @Override
  protected boolean isFinished() {
    if (Robot.splinePercentage >= activationPoint) {
      return true;
    }
    return false;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
