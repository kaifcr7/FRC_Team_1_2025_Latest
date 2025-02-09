// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.CANdleSubsystem;

import edu.wpi.first.wpilibj.Timer;

public class ElevatorFixedLocationCommand extends Command 
{
  CANdleSubsystem m_candle;
  
  double lastTimeStamp = 0;
  double executeTimeStamp = 0;
  double isFinishedTimeStamp = 0;

  private final ElevatorSubsystem ElevatorSubsystem;
  private final Double  Distance;
  final double iLimitDistance = 1;  /*limit ` distance*/
  double RPSsetpoint = 0;
  double errorSum = 0;
  double lastError = 0;
  double setpointDistance= 0;
  double errorSumDistance = 0;
  double lastErrorDistance = 0;
  double TotalDistance= 0;
  double  startingDistance = 0;
  double   currentDistancePosition = 0;
  double   limitSpeed = 0.5;

  double outputSpeed = 0;
  boolean isForward = true;
  double errorDistance = 7;
  int i = 0;

 
  public ElevatorFixedLocationCommand(ElevatorSubsystem Elevator, Double Distance) 
  {
    this.ElevatorSubsystem = Elevator;
    this.Distance = Distance;
    
    addRequirements(Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

    
    ElevatorSubsystem.setElevatorBrake();
    

    lastTimeStamp = Timer.getFPGATimestamp();
       
    errorSumDistance = 0;
    lastErrorDistance =0;
    setpointDistance = this.Distance*(120/9/25.4);  //motor revolutions per inch

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
  
    /*************************/
    /*kP                     */
    /*************************/
    ;
    currentDistancePosition = ElevatorSubsystem.getPosition();

   
    errorDistance = (setpointDistance - currentDistancePosition);  


    if (Math.abs(errorDistance) < 1)  //deadband  
    {
         errorDistance = 0;
   
    }





    /*************************/
    /*kI                    */
    /*************************/
    double dt  = Timer.getFPGATimestamp() - lastTimeStamp;
    //  errorSum = 0;
    // if error is < 1 meter than errorSum is calcuated for kI

    if (Math.abs(errorDistance) < iLimitDistance)   //only executes when error less than 1 meter    
    {
      errorSumDistance += errorDistance * dt;

    }

    /*************************/
    /*kD                     */
    /*************************/
    double errorRateDistance = (errorDistance - lastErrorDistance)/dt;

    /*************************/
    /*kP kI kD Calculations  */
    /*************************/

    double outputSpeed =  ElevatorSubsystem.kP * errorDistance + ElevatorSubsystem.kI * errorSumDistance + ElevatorSubsystem.kD * errorRateDistance;
   
        
    if (outputSpeed > limitSpeed  )     
    {
      outputSpeed = limitSpeed ;
    }
    else if (outputSpeed < -limitSpeed ) 
    {
      outputSpeed = -limitSpeed ;
   
    }
        
    ElevatorSubsystem.setElevatorSpeed(outputSpeed);
    ElevatorSubsystem.setElevatorBrake();

    SmartDashboard.putNumber("Elevator OutputSpeed" ,outputSpeed);
    SmartDashboard.putNumber("Elevator setPoint inches" ,setpointDistance);
    SmartDashboard.putNumber("Elevator our distance motor" , currentDistancePosition);
       
    /*setup for next execution */
    lastTimeStamp = Timer.getFPGATimestamp();
    lastErrorDistance= errorDistance;
    
  }  
    

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    isFinishedTimeStamp = Timer.getFPGATimestamp();
    ElevatorSubsystem.lastPosition = this.currentDistancePosition;
    ElevatorSubsystem.setElevatorSpeed(0);
    ElevatorSubsystem.setElevatorBrake();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
   
      if (Math.abs(errorDistance) < 1.0 )
    {
          ElevatorSubsystem.setElevatorSpeed(0.0);
          ElevatorSubsystem.setElevatorDisable();
         
          return true;
    }        
    else
    {    
      
    return false;
    }
  }
}
