package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.util.SubsystemCommand;
import frc.robot.util.SubsystemModule;

/*
 * public class Intake extends SubsystemModule {
 * 
 * public Spark leftIntake = new Spark(RobotMap.p_leftIntake); public Spark
 * rightIntake = new Spark(RobotMap.p_rightIntake);
 * 
 * public DoubleSolenoid intakePivot = new
 * DoubleSolenoid(RobotMap.p_intakePivotA, RobotMap.p_intakePivotB);
 * 
 * public Intake(){ registerCommands(); }
 * 
 * @Override public void initDefaultCommand() {
 * 
 * }
 * 
 * @Override public void run() {
 * 
 * }
 * 
 * @Override public void registerCommands() {
 * 
 * new SubsystemCommand(this.registeredCommands, "intake_in", 0) {
 * 
 * @Override public void initialize() { Robot.intake.leftIntake.set(0.8);
 * Robot.intake.rightIntake.set(-0.8); }
 * 
 * @Override public void end() { Robot.intake.leftIntake.set(0);
 * Robot.intake.rightIntake.set(0); }
 * 
 * @Override public boolean isFinished() { return false; } };
 * 
 * new SubsystemCommand(this.registeredCommands, "intake_in_start", 0) {
 * 
 * @Override public void initialize() { Robot.intake.leftIntake.set(0.8);
 * Robot.intake.rightIntake.set(-0.8); }
 * 
 * @Override public boolean isFinished() { return true; } };
 * 
 * new SubsystemCommand(this.registeredCommands, "intake_out", 0) {
 * 
 * @Override public void initialize() { Robot.intake.leftIntake.set(-0.8);
 * Robot.intake.rightIntake.set(0.8); }
 * 
 * @Override public void end() { Robot.intake.leftIntake.set(0);
 * Robot.intake.rightIntake.set(0); }
 * 
 * @Override public boolean isFinished() { return false; } };
 * 
 * new SubsystemCommand(this.registeredCommands, "intake_out_start", 0) {
 * 
 * @Override public void initialize() { Robot.intake.leftIntake.set(-0.8);
 * Robot.intake.rightIntake.set(0.8); }
 * 
 * @Override public boolean isFinished() { return true; } };
 * 
 * new SubsystemCommand(this.registeredCommands, "intake_stop", 0) {
 * 
 * @Override public void initialize() { Robot.intake.leftIntake.set(0);
 * Robot.intake.rightIntake.set(0); }
 * 
 * @Override public boolean isFinished() { return true; } };
 * 
 * new SubsystemCommand(this.registeredCommands, "raise_intake", 0) {
 * 
 * @Override public void initialize() {
 * Robot.intake.intakePivot.set(DoubleSolenoid.Value.kReverse); } };
 * 
 * new SubsystemCommand(this.registeredCommands, "lower_intake", 0) {
 * 
 * @Override public void initialize() {
 * Robot.intake.intakePivot.set(DoubleSolenoid.Value.kForward); } }; }
 * 
 * }
 */