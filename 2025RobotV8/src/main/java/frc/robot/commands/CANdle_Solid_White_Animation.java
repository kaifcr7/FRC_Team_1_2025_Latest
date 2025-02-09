// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANdleSubsystem;

public class CANdle_Solid_White_Animation extends Command {

  CANdleSubsystem m_candle;
  public CANdle_Solid_White_Animation(CANdleSubsystem candle) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_candle = candle;
    addRequirements(candle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_candle.CANdle_init();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_candle.CANdle_Solid_White();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
     m_candle.CANdle_Default();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
