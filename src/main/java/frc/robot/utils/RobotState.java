// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/**
 * Enums for robot status (set with the DPAD),
 * helps the robot determine what shooting status it should be in
 * and autodrive align with the correct field element
 * @see frc.robot.commands.MultiSubsystem.Shooting
 * @see frc.robot.commands.DriveTrain.AutoDrive
 */
public class RobotState {

  /**
   * Places where the robot will want to be at
   */
  public static enum RobotLocations {
    AMP,
    SPEAKER,
    INTAKE,
    UNKNOWN,
  }

  /**
   * States the robot will be in involving its intake and shooter
   */
  public static enum ShootingState {
    AMP,
    SPEAKER,
    INTAKE,
    HANDSHAKE,
  }

  private static RobotLocations currentLocation = RobotLocations.UNKNOWN;

  /**
   * Gets what the software thinks the driver intends to do with the robot
   * @return
   */
  public static RobotLocations getRobotLocation() {
    return currentLocation;
  }

  /**
   * Uses the Dpad to set driver intentions
   * @param location
   */
  public static void setRobotLocation(RobotLocations location) {
    currentLocation = location;
  }
}
