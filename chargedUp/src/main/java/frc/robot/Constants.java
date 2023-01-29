// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class DrivetrainConstants{
    public static final int[] kDrivetrainCANIDs = new int[] {5,6,3,4};

    public static final boolean kRightInverted = true;
    public static final boolean kLeftInverted = false;

    // todo: update equation
    public static final double kTicksToFeat = 0;
    public static final double kStopMotors = 0;

    //PID Controlls for Forawrds and Backwards
    public static final double kMoveP = 25;
    public static final double kMoveI = 0;
    public static final double kMoveD = 0;
    public static final double kMoveTolerance = 1;

    //PID Controlls for Turning
    public static final double kTurnP = 0;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;
    public static final double kTurnTolerance = 0;
  }

  public static class ArmConstants{
    public static final int[] armCANIDs = new int[] {1, 2, 0, 0, 0};
    public static final double kArmPositionConversion = 0;

    //Arm PID Values (Tune PID Before Feedforward)
    public static final double kArmP = 8;
    public static final double kArmI = 0;
    public static final double kArmD = 0;
    public static final double kArmTolerance = 0.5;

    //Trapazoidal Motion Profiling
    public static final double kArmMaxVelocity = 100;
    public static final double kArmMaxAcceleration = 100;
    
  }
}
