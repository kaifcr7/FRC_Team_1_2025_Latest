// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbStickCommand extends Command
{
 
  private final ClimbSubsystem ClimbSubsystem;
  private final Supplier<Double> speedFunction;
                   

  /*********************************************************/
  /*   ClimbStickCmd  constructor   During teleop         */
  /*********************************************************/
  public ClimbStickCommand(ClimbSubsystem  ClimbSubsystem,
                             Supplier<Double> speedFunction)
                              
  {
    this. ClimbSubsystem=  ClimbSubsystem;
    this.speedFunction = speedFunction;    //Y axis on xbox controller 
    addRequirements(ClimbSubsystem);
  }
  /*********************************************************/
  /*   initlize runs after constructor automatically       */
  /*********************************************************/
  @Override
  public void initialize() 
  {  
   
    SmartDashboard.putString("climb controller", "started");
  }
  /*********************************************************/
  /*   Execute runs isFinished method returns a true       */
  /*********************************************************/

  @Override
  public void execute() 
  {
      double speed = 0;
      double UpDownSpeed = (-speedFunction.get()) * -1.0;   //multiple by -1  ... or remove negative
      if((UpDownSpeed <.05) && (UpDownSpeed > -.05))
      {
        speed =  0;
      }
      else
      { 
         speed = speedFunction.get();
      }

      SmartDashboard.putNumber("Climb controller speed", speedFunction.get());
      ClimbSubsystem.setClimbSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) 
  {

    System.out.println("Climb JoytStickended");
  }

  /*********************************************************/
  /*   isFinished always returns false so CMD during telop */
  /*        never ends                                     */
  /*********************************************************/
  @Override
  public boolean isFinished() {
    return false;
  }
}

