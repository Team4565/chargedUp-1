// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants;

public class autoBalance extends CommandBase {
  /** Creates a new autoBalance. */
  public boolean goesForward;
  private final DrivetrainSubsystem m_DrivetrainSubsystem;

  public autoBalance(DrivetrainSubsystem drivetrainSubsystem, boolean Forwards) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DrivetrainSubsystem = drivetrainSubsystem;
    goesForward = Forwards;
    
    
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(goesForward) { 
      if(m_DrivetrainSubsystem.getPosition() < 1.3)
      m_DrivetrainSubsystem.setRaw(.2, 0);
      else {
        m_DrivetrainSubsystem.setRaw(0, .2);
      }
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
