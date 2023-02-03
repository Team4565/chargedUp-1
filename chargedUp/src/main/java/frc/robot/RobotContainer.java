// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.autoBalance;
import frc.robot.commands.targetFinding;
import frc.robot.commands.DrivetrainPID.MovePID;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import javax.swing.plaf.basic.BasicOptionPaneUI.ButtonActionListener;

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();


  private final Command m_autoBalance =
  new autoBalance(m_drivetrainSubsystem);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController m_driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    //Moves Robot Using Joysticks
    m_drivetrainSubsystem.setDefaultCommand(new RunCommand(() ->
     m_drivetrainSubsystem.setRaw(m_driverController.getLeftY(), m_driverController.getRightX()), m_drivetrainSubsystem));
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    // m_driverController.a().whileTrue(new RunCommand(() ->SmartDashboard.putString("button pressed", "a")) );
    // m_driverController.b().whileTrue(new RunCommand(() ->SmartDashboard.putString("button pressed", "b")) );

    // m_driverController.x().whileTrue(new RunCommand(() ->SmartDashboard.putString("button pressed", "x")) );

    // m_driverController.y().whileTrue(new RunCommand(() ->SmartDashboard.putString("button pressed", "y")) );

    // //D-Pad buttons
    // m_driverController.povUp().whileTrue(new RunCommand(() -> SmartDashboard.putString("button pressed", "Up")) );

    // m_driverController.povDown().whileTrue(new RunCommand(() -> SmartDashboard.putString("button pressed", "Down")) );
    
    // m_driverController.povRight().whileTrue(new RunCommand(() -> SmartDashboard.putString("button pressed", "Right")) );

    // m_driverController.povLeft().whileTrue(new RunCommand(() -> SmartDashboard.putString("button pressed", "Left")) );


    // m_driverController.leftBumper().whileTrue(new RunCommand(() -> SmartDashboard.putString("button pressed", "leftBumper")) );

    // m_driverController.rightBumper().whileTrue(new RunCommand(() -> SmartDashboard.putString("button pressed", "rightBumper")) );

    // m_driverController.leftTrigger().whileTrue(new RunCommand(() -> SmartDashboard.putString("button pressed", "leftTrigger")) );

    // m_driverController.rightTrigger().whileTrue(new RunCommand(() -> SmartDashboard.putString("button pressed", "rightTrigger")) );

    // m_driverController.start().whileTrue(new RunCommand (() -> SmartDashboard.putString("button pressed", "startButton")) );

    //Spins Motor if April Tags are Recognized for 20 Ticks
    new JoystickButton(m_driverController, XboxController.Button.kA.value).
        onTrue(new targetFinding(m_drivetrainSubsystem, m_visionSubsystem));

    new JoystickButton(m_driverController, XboxController.Button.kB.value).
        onTrue(new autoBalance(m_drivetrainSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
