// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShoulderSubsystem;

import edu.wpi.first.wpilibj.Timer;

public class ShoulderAngleCommand extends Command 
{
  double lastTimeStamp = 0;
  double executeTimeStamp = 0;
  double isFinishedTimeStamp = 0;

  private final ShoulderSubsystem Shoulder;
  private final Double  Distance;
  final double iLimitDistance = 0.02;  /*limit ` distance*/
  double RPSsetpoint = 0;
  double errorSum = 0;
  double lastError = 0;
  double setpointDistance= 0;
  double errorSumDistance = 0;
  double lastErrorDistance = 0;
  double TotalDistance= 0;
  double  startingDistance = 0;
  double   currentDistancePosition = 0;
  double   limitSpeed = 0.2;

  double outputSpeed = 0;
  boolean isForward = true;
  double errorDistance = 0;
  int i = 0;
 

  /** Creates a new ShooterPitchFixedLocationCommand. */
  public ShoulderAngleCommand(ShoulderSubsystem Shoulder, Double Distance) 
  {
    this.Shoulder = Shoulder;
    this.Distance = Distance;
    
    addRequirements(Shoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

    Shoulder.setShoulderBrake();

    lastTimeStamp = Timer.getFPGATimestamp();
      
    errorSumDistance = 0;
    lastErrorDistance =0;
    setpointDistance = this.Distance;  //motor revolutions per inch

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    /*************************/
    /*kP and kV                   */
    /*************************/

    currentDistancePosition = Shoulder.getPosition();
    
    double feedForwardAngle = (Math.sin((currentDistancePosition + 0)*3.1415/180));

   
    errorDistance = (setpointDistance - currentDistancePosition);  


    if (Math.abs(errorDistance) < .001)  //deadband  
    {
       errorDistance = 0;
   
    }

    /*************************/
    /*kI                    */
    /*************************/
    double dt  = Timer.getFPGATimestamp() - lastTimeStamp;
    errorSum = 0;
   
    if (Math.abs(errorDistance) < iLimitDistance)   //.01
    {
      errorSumDistance += errorDistance * dt;

    }

    // /*************************/
    // /*kD                     */
    // /*************************/
    double errorRateDistance = (errorDistance - lastErrorDistance)/dt;


    /*************************/
    /*kP kI kD Calculations  */
    /*************************/

    double outputSpeed =  Shoulder.kV*feedForwardAngle + Shoulder.kP * errorDistance + Shoulder.kI * errorSumDistance + Shoulder.kD * errorRateDistance;
   
        
    if (outputSpeed > limitSpeed  )  
    {
      outputSpeed = limitSpeed ;
    
    }
    if (outputSpeed < -limitSpeed ) 
    {
        outputSpeed = -limitSpeed;
   
    }


    Shoulder.setShoulderSpeed(outputSpeed);

    
    SmartDashboard.putNumber("Shoulder setPoint inches" ,setpointDistance);  
    SmartDashboard.putNumber("Shoulder OutputSpeed" ,outputSpeed);

       
    /*setup for next execution */
    lastTimeStamp = Timer.getFPGATimestamp();
    lastErrorDistance= errorDistance;
  }  
    
  @Override
  public void end(boolean interrupted) {

    isFinishedTimeStamp = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("ShooterPitch isFinished", isFinishedTimeStamp);
    Shoulder.lastPosition = this.currentDistancePosition;
    Shoulder.setShoulderSpeed(0);
    Shoulder.setShoulderBrake();
        
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
   
     
    return false;
  }
}
