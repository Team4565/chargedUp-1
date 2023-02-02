// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants;

public class MoveCommand extends CommandBase {
  /** Creates a new MoveCommand. */
  public int toDistance;
  public boolean goesForward;
  private final DrivetrainSubsystem m_DrivetrainSubsystem;

  public MoveCommand(DrivetrainSubsystem drivetrainSubsystem, Integer Distance, boolean Forwards) {
    m_DrivetrainSubsystem = drivetrainSubsystem;
    addRequirements(m_DrivetrainSubsystem);
    toDistance = Distance;
    goesForward = Forwards;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DrivetrainSubsystem.resetEncoderValue();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (goesForward) {
    //   goForward(toDistance);
    // } else if (!goesForward) {
    //  goBackwards(toDistance);
    // } else {
    //   m_DrivetrainSubsystem.setRaw(0.00, 0.00);

    // }
    if (goesForward) {
      if (m_DrivetrainSubsystem.getAvgEncocderDistance() < toDistance)
        m_DrivetrainSubsystem.setRaw(-0.5, 0);
      else{
        m_DrivetrainSubsystem.setRaw(0.00, 0.00);
      }
    } else if (!goesForward) {
      if (m_DrivetrainSubsystem.getAvgEncocderDistance() > toDistance)
        m_DrivetrainSubsystem.setRaw(0.5, 0);
    } else {
      m_DrivetrainSubsystem.setRaw(0.00, 0.00);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return goesForward ?  (m_DrivetrainSubsystem.getAvgEncocderDistance() >= toDistance) : (m_DrivetrainSubsystem.getAvgEncocderDistance() <= toDistance);
  }


  public void goForward(int distance){
System.out.println("FORWARD" + distance);
  }

  public void goBackwards(int distance){
System.out.println("BACKWARDS" + distance);
  }
}
        
      
    

  

