// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SensorSubsystem extends SubsystemBase {
  /** Creates a new SensorSubsystem. */
  private final AnalogInput m_rangeFinder; 


  public SensorSubsystem() {
    m_rangeFinder = new AnalogInput(0);
  }

  @Override
  public void periodic() {
    // try (// This method will be called once per scheduler run
      double distanceVoltage = m_rangeFinder.getValue();
      SmartDashboard.putNumber("distance voltage", distanceVoltage);
      double voltage_scale_factor = 5/RobotController.getVoltage5V();
      double distanceInches = distanceVoltage * voltage_scale_factor * 0.0492;
      SmartDashboard.putNumber("distance inches", distanceInches);
    // }
    // catch(Exception e) { System.out.println("error with the sensor");}
   }
}
