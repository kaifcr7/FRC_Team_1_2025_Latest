// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Analog0Subsystem extends SubsystemBase {
 
  AnalogInput  analogIntake = new AnalogInput(0);
  public Analog0Subsystem()   {    }

public Boolean isAnalogIntakeSwitchClosed()
{
  if(analogIntake.getVoltage() < .13)
  {
    //this true just says that the switch is closed nothing else
    //will turn green....
    return true;
  }
   return false;
}

public double isAnalogEyeGetVoltage()
{
  return analogIntake.getVoltage();
   
}
  @Override
  public void periodic() 
  
  {
   SmartDashboard.putNumber("Analog0 Eye",this.isAnalogEyeGetVoltage());
  }
}
