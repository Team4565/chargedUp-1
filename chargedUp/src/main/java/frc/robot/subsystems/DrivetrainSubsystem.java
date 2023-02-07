// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */
 


  private static DifferentialDrive diffDrive;
 
  private final WPI_TalonFX leftLead;
  private final WPI_TalonFX rightLead; 
  private final WPI_TalonFX leftFollower;
  private final WPI_TalonFX rightFollower;



  public DrivetrainSubsystem() {
    leftLead = new WPI_TalonFX(2);
    rightLead = new WPI_TalonFX(4);
    leftFollower = new WPI_TalonFX(3);
    rightFollower = new WPI_TalonFX(5);

    leftLead.setInverted(true);
    leftFollower.setInverted(true);
    leftFollower.follow(leftLead);

    
    rightFollower.follow(rightLead);

    //invert one of the leads :)

    diffDrive = new DifferentialDrive(rightLead,leftLead);
  }


   
  public void setRaw(double driveValue, double turnValue){
    diffDrive.arcadeDrive(driveValue, turnValue);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
}
