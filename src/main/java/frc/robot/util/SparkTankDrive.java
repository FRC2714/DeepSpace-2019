package frc.robot.util;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SparkTankDrive {

    //DriveTrain Motors
    private CANSparkMax lMotor0 = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax lMotor1 = new CANSparkMax(1, MotorType.kBrushless);
    private CANSparkMax lMotor2 = new CANSparkMax(2, MotorType.kBrushless);
    private CANSparkMax rMotor0 = new CANSparkMax(3, MotorType.kBrushless);
    private CANSparkMax rMotor1 = new CANSparkMax(4, MotorType.kBrushless);
    private CANSparkMax rMotor2 = new CANSparkMax(5, MotorType.kBrushless);

    //PID Controllers
    private CANPIDController lPidController = lMotor0.getPIDController();
    private CANPIDController rPidController = rMotor0.getPIDController();

    //MAX Encoders
    private CANEncoder lEncoder = lMotor0.getEncoder();
    private CANEncoder rEncoder = rMotor0.getEncoder();

    //PID Coefficients
    public double lKP, lKI, lKIS, lKD, kFF, kMaxOutput, kMinOutput, maxRPM;


    public SparkTankDrive () {
      
    }
  
      // PID coefficients
      kP = 5e-5; 
      kI = 1e-6;
      kD = 0; 
      kIz = 0; 
      kFF = 0; 
      kMaxOutput = 1; 
      kMinOutput = -1;
      maxRPM = 5700;
  
      // set PID coefficients
      m_pidController.setP(kP);
      m_pidController.setI(kI);
      m_pidController.setD(kD);
      m_pidController.setIZone(kIz);
      m_pidController.setFF(kFF);
      m_pidController.setOutputRange(kMinOutput, kMaxOutput);
  
      // display PID coefficients on SmartDashboard
      SmartDashboard.putNumber("P Gain", kP);
      SmartDashboard.putNumber("I Gain", kI);
      SmartDashboard.putNumber("D Gain", kD);
      SmartDashboard.putNumber("I Zone", kIz);
      SmartDashboard.putNumber("Feed Forward", kFF);
      SmartDashboard.putNumber("Max Output", kMaxOutput);
      SmartDashboard.putNumber("Min Output", kMinOutput);
    }
  
    @Override
    public void teleopPeriodic() {
      // read PID coefficients from SmartDashboard
      double p = SmartDashboard.getNumber("P Gain", 0);
      double i = SmartDashboard.getNumber("I Gain", 0);
      double d = SmartDashboard.getNumber("D Gain", 0);
      double iz = SmartDashboard.getNumber("I Zone", 0);
      double ff = SmartDashboard.getNumber("Feed Forward", 0);
      double max = SmartDashboard.getNumber("Max Output", 0);
      double min = SmartDashboard.getNumber("Min Output", 0);
  
      // if PID coefficients on SmartDashboard have changed, write new values to controller
      if((p != kP)) { m_pidController.setP(p); kP = p; }
      if((i != kI)) { m_pidController.setI(i); kI = i; }
      if((d != kD)) { m_pidController.setD(d); kD = d; }
      if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
      if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
      if((max != kMaxOutput) || (min != kMinOutput)) { 
        m_pidController.setOutputRange(min, max); 
        kMinOutput = min; kMaxOutput = max; 
      }
  
      /**
       * PIDController objects are commanded to a set point using the 
       * SetReference() method.
       * 
       * The first parameter is the value of the set point, whose units vary
       * depending on the control type set in the second parameter.
       * 
       * The second parameter is the control type can be set to one of four 
       * parameters:
       *  com.revrobotics.ControlType.kDutyCycle
       *  com.revrobotics.ControlType.kPosition
       *  com.revrobotics.ControlType.kVelocity
       *  com.revrobotics.ControlType.kVoltage
       */
      double setPoint = m_stick.getY()*maxRPM;
      m_pidController.setReference(setPoint, ControlType.kVelocity);
      
      SmartDashboard.putNumber("SetPoint", setPoint);
      SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
    }

}