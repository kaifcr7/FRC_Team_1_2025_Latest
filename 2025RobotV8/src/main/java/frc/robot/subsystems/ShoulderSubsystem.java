// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.controls.NeutralOut;
//import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

 public class ShoulderSubsystem extends SubsystemBase 
{

  private final TalonFX m_shoulder = new TalonFX(17);
  
  private final CANcoder m_cancoder = new CANcoder(18);

  public double RPMultiplier;
  public double kP = 0.01;
  public double kI = 0.005;
  public double kD = 0.00000;
  public double kV =  0.1;
  public String subSystem = "Shoulder";
  public double lastPosition = 0;
  //torque

  public TalonFXConfiguration configs = new TalonFXConfiguration();
  public double kFF, kMaxOutput, kMinOutput, maxRPM;

  
  private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0.0);
  //private final VelocityTorqueCurrentFOC m_torqueVelocity = new VelocityTorqueCurrentFOC(0);  
  private final NeutralOut m_brake = new NeutralOut();
    
  public  ShoulderSubsystem()
  {

    m_shoulder.setNeutralMode(NeutralModeValue.Brake);
   

    configs = new TalonFXConfiguration();
    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = kP; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = kI; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = kD; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = kV; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;
    
    /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    configs.Slot1.kP = kP; // An error of 1 rotation per second results in 5 amps output
    configs.Slot1.kI = kI; // An error of 1 rotation per second increases output by 0.1 amps every second
    configs.Slot1.kD = kD; // A change of 1000 rotation per second squared results in 1 amp output
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;
    configs.Feedback.FeedbackRemoteSensorID = m_cancoder.getDeviceID();
    configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {

      status = m_shoulder.getConfigurator().apply(configs);
      
      if (status.isOK()) break;
    }
    
     m_shoulder.setControl(m_brake);
     m_shoulder.setPosition(0);
     var rotorPosSignal = m_shoulder.getRotorPosition();
    
     rotorPosSignal.refresh();
     m_shoulder.setControl(m_brake);
     m_shoulder.setNeutralMode(NeutralModeValue.Brake);
     if(!status.isOK()) {
      System.out.println("Could not apply Shoulder configs, error code: " + status.toString());
    }
      
  }

  public void setShoulderRPS(double desiredRoationsPerSecond)
  {
      m_shoulder.setControl(m_voltageVelocity.withVelocity(desiredRoationsPerSecond));
  }

  public TalonFX getShoulderMotor()
  {
    return m_shoulder;
  } 

  public void setShoulderSpeed(double percentOutput)
  {
    m_shoulder.set(percentOutput);  
  }

  public double getPosition()
  {    
    return (m_cancoder.getAbsolutePosition().getValueAsDouble());
  }
 
  public void setShoulderCoast()
  {    
    m_shoulder.setNeutralMode(NeutralModeValue.Coast);        
  }

  public void setShoulderBrake()
  {
         m_shoulder.setNeutralMode(NeutralModeValue.Brake );     
  }
 
   /**
   * @return
   */
  public double getShoulderVelocity()
  {  
    return m_shoulder.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() 
  {  
    SmartDashboard.putNumber("Shoulder CANCoder position", m_cancoder.getAbsolutePosition().getValueAsDouble()); 
     
  }
}

