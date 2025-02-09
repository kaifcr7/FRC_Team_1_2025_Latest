// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

 public class ElevatorSubsystem extends SubsystemBase 
{

  private final TalonFX m_elevator = new TalonFX(15);
  private final TalonFX m_elevator2 = new TalonFX(16);
  private final DigitalInput DIO = new DigitalInput(0);
  
  public double kP = 0.01;
  public double kI = 0.005;
  public double kD = 0.00000;
  public double kV =  0.0;
  public String subSystem = "Elevator";
  public double lastPosition = 0;
  //torque

  public TalonFXConfiguration configs = new TalonFXConfiguration();
  public double kFF, kMaxOutput, kMinOutput, maxRPM;

  
  private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0.0);
 
  
  private final NeutralOut m_brake = new NeutralOut();
    
  public  ElevatorSubsystem()
  {

    m_elevator.setNeutralMode(NeutralModeValue.Brake);
    m_elevator2.setNeutralMode(NeutralModeValue.Brake);
    m_elevator2.setControl(new Follower(15, true ));
   

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

    
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {

      status = m_elevator.getConfigurator().apply(configs);
      
      if (status.isOK()) break;
    }
    
    StatusCode status1 = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {

      status1 = m_elevator2.getConfigurator().apply(configs);
      
      if (status1.isOK()) break;
    }


     m_elevator.setControl(m_brake);
     m_elevator2.setControl(m_brake);
     m_elevator.setPosition(0);
     

     var rotorPosSignal = m_elevator.getRotorPosition();
  
     rotorPosSignal.refresh();
     m_elevator.setControl(m_brake);
     m_elevator2.setControl(m_brake);

     m_elevator.setNeutralMode(NeutralModeValue.Brake );
     m_elevator2.setNeutralMode(NeutralModeValue.Brake );

    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
    if(!status1.isOK()) {
      System.out.println("Could not apply configs, error code: " + status1.toString());
    }

    m_elevator.getPosition().setUpdateFrequency(20);
  
  }


  public void setElevatorRPS(double desiredRoationsPerSecond)
  {
   
    m_elevator.setControl(m_voltageVelocity.withVelocity(desiredRoationsPerSecond));
    
  }
  public TalonFX getElevatorMotor()
  {
    return m_elevator;
  } 

  public void setElevatorDisable()
  {
    m_elevator.disable();
  } 

   
  public void setElevatorSpeed(double percentOutput)
  {
    m_elevator.set(percentOutput);
    m_elevator2.set(-percentOutput);
    SmartDashboard.putNumber("Elevator Speed Setpoint", percentOutput);

  }
public double getPosition()
  {
  	
    var rotorPosSignal = m_elevator.getRotorPosition();

    return(rotorPosSignal.getValueAsDouble());

  }
   
  public void setElevatorCoast()
  {
    
    m_elevator.setNeutralMode(NeutralModeValue.Coast);
    m_elevator2.setNeutralMode(NeutralModeValue.Coast);
            
  }

  public void setElevatorBrake()
  {
         m_elevator.setNeutralMode(NeutralModeValue.Brake ); 
         m_elevator2.setNeutralMode(NeutralModeValue.Brake ); 
  
  
  }
  
  
  

  //****************************************************************/
  //*             getVelocity                          */
  //************************************************************** */
  /**
   * @return
   */
  public double getElevatorVelocity()
  {
  
    return m_elevator.getVelocity().getValueAsDouble();
  }

 

  @Override
  public void periodic() 
  {  


    if(!DIO.get())
    {
      m_elevator.setPosition(0);
      lastPosition = 0; 
    
    }

    SmartDashboard.putNumber("Elevator position" , this.getPosition());
    SmartDashboard.putNumber("Elevator Velocity", this.getElevatorVelocity());
    SmartDashboard.putNumber("Elevator Current", m_elevator.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Elevator2 Current", m_elevator.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putBoolean("Elevator DIO",  DIO.get());
  }


}
