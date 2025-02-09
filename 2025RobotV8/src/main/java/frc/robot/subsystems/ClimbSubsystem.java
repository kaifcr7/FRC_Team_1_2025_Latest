// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase 
{

  int countF = 0;
  int countR = 0;
  

  TalonSRX _ClimbRightMotor = new TalonSRX(26);

      /** electic brake during neutral */
	final NeutralMode kBrakeDurNeutral = NeutralMode.Brake;



  /*********************************************************/
  /*   ClimbSubSystem constructor                         */
  /*********************************************************/
  public ClimbSubsystem() 
  {
    
    TalonSRXConfiguration configs = new TalonSRXConfiguration();

     _ClimbRightMotor.configAllSettings(configs);
    
     
	   _ClimbRightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
		
     
	  _ClimbRightMotor.setInverted(true);
		
    
		_ClimbRightMotor.setNeutralMode(kBrakeDurNeutral);
    
     _ClimbRightMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 200); 
       
   }
  
  public void setClimbSpeed(double speed)
  {
    _ClimbRightMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() 
  {

    SmartDashboard.putNumber("Climb Out Percent", _ClimbRightMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("Climb Current", _ClimbRightMotor.getSupplyCurrent());

  }
}
