// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderJoyStickCommand extends Command
{
  
  private final ShoulderSubsystem ShoulderSubsystem;
  private final Supplier<Double> speedFunction;
  private double realTimeSpeed=0;
                   

  public ShoulderJoyStickCommand( ShoulderSubsystem  ShoulderSubsystem,
                                      Supplier<Double> speedFunction)
                              
  {
    this. ShoulderSubsystem=  ShoulderSubsystem;
    this.speedFunction = speedFunction;    //Y axis on xbox controller 
    addRequirements(ShoulderSubsystem);

  }
  
  @Override
  public void initialize() 
  {
      
  }
 

  @Override
  public void execute() 
  {
    double realTimeSpeed = speedFunction.get();   //Y  is now positive up
                                                  //negative for down....
    SmartDashboard.putNumber("Shoulder JoyStick Enabled",  realTimeSpeed);

    //Joy Stick ramping
     realTimeSpeed = realTimeSpeed * Math.abs(realTimeSpeed);

    if((realTimeSpeed <.05) && (realTimeSpeed > -.05))
    {
        realTimeSpeed =  0;
    }
           
    double motor = realTimeSpeed;

    double currentDistancePosition = ShoulderSubsystem.getPosition();
    double max = -0.25;
    double min = -0.02;

       
    // - 0.251       < -0.25               //.25
    if(currentDistancePosition < max && motor < 0)
    {
         ShoulderSubsystem.setShoulderSpeed(0.001);
    }
    //if reached min and joystick still trying to go down stop the whole thing...
    // //            -0.04 > -.05            && -.01 joystick
    else if (currentDistancePosition > min && motor > 0)
    {
          ShoulderSubsystem.setShoulderSpeed(0.001);
    }
    else  ShoulderSubsystem.setShoulderSpeed(motor*0.1);

  }

  @Override
  public void end(boolean interrupted) 
  {
  
  }

  @Override
  public boolean isFinished() 
  {
    
      return false;
  }
}
