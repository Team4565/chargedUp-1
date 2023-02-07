// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */
  private static CANSparkMax leftLead;
  private static CANSparkMax rightLead;
  private static CANSparkMax leftFollower1;
  private static CANSparkMax rightFollower1;
  private static CANSparkMax leftFollower2;
  private static CANSparkMax rightFollower2;


  private static RelativeEncoder encoderLeftLead;
  private static RelativeEncoder encoderRightLead;

  private static DifferentialDrive diffDrive;
  public DifferentialDrivetrainSim m_drivetrainSimulator;



  public DrivetrainSubsystem() {

    leftLead = new CANSparkMax(DrivetrainConstants.kDrivetrainCANIDs[3], MotorType.kBrushless);
    rightLead = new CANSparkMax(DrivetrainConstants.kDrivetrainCANIDs[0], MotorType.kBrushless);

    leftFollower1 = new CANSparkMax(DrivetrainConstants.kDrivetrainCANIDs[4], MotorType.kBrushless);
    rightFollower1 = new CANSparkMax(DrivetrainConstants.kDrivetrainCANIDs[1], MotorType.kBrushless);

    leftFollower2 = new CANSparkMax(DrivetrainConstants.kDrivetrainCANIDs[5], MotorType.kBrushless);
    rightFollower2 = new CANSparkMax(DrivetrainConstants.kDrivetrainCANIDs[2], MotorType.kBrushless);

    leftLead.setIdleMode(IdleMode.kBrake);
    rightLead.setIdleMode(IdleMode.kBrake);

    leftFollower1.setIdleMode(IdleMode.kBrake);
    rightFollower1.setIdleMode(IdleMode.kBrake);

    leftFollower2.setIdleMode(IdleMode.kBrake);
    rightFollower2.setIdleMode(IdleMode.kBrake);

    encoderLeftLead = leftLead.getEncoder();
    
    leftLead.setInverted(DrivetrainConstants.kLeftInverted);
    // todo: uncomment for conversion
    // encoderLeftLead.setPositionConversionFactor(DrivetrainConstants.kTicksToFeat);
    // encoderLeftLead.setInverted(DrivetrainConstants.kLeftInverted);

    leftFollower1.follow(leftLead);
    leftFollower2.follow(leftLead);


    encoderRightLead = rightLead.getEncoder();

    // todo: uncomment for conversion
    // encoderRightLead.setPositionConversionFactor(DrivetrainConstants.kTicksToFeat);
    rightLead.setInverted(DrivetrainConstants.kRightInverted);

    rightFollower1.follow(rightLead);
    rightFollower2.follow(rightLead);
    
     
    diffDrive = new DifferentialDrive(leftLead, rightLead);
    
    if (RobotBase.isSimulation()) { // If our robot is simulated
      // This class simulates our drivetrain's motion around the field.
      m_drivetrainSimulator =
          new DifferentialDrivetrainSim(
            // todo: will fix :)
              DCMotor.getNEO(3),
              DrivetrainConstants.kgearing,
              DrivetrainConstants.kMOI,
              DrivetrainConstants.kMass,
              DrivetrainConstants.kwheelRadius,
              DrivetrainConstants.ktrackWidth,
              // The standard deviations for measurement noise:
              // x and y:          0.001 m
              // heading:          0.001 rad
              // l and r velocity: 0.1   m/s
              // l and r position: 0.005 m
              VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));}

             
  }


   
  public void setRaw(double driveValue, double turnValue){
    diffDrive.arcadeDrive(driveValue, turnValue);
  }

  public double getPosition(){
    return encoderLeftLead.getPosition();
  }

  public void resetPosition(){
    encoderLeftLead.setPosition(0);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void SmartDashboardCalls(){
    SmartDashboard.putNumber("Drivetrain Position", this.getPosition());
  }
  
}
