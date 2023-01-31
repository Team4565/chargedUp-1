// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final CANSparkMax m_intakeRoller;
  private final RelativeEncoder m_rollerEncoder;
  public IntakeSubsystem() {
    m_intakeRoller = new CANSparkMax(IntakeConstants.kIntakeCANIDs[0], MotorType.kBrushless);
    m_rollerEncoder = m_intakeRoller.getEncoder();
  }

  public void spinRoller(double speed){
    m_intakeRoller.set(speed);
  }

  public void stopRoller(){
    m_intakeRoller.set(0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
