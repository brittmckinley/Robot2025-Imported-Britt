// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;

public class AccelerationConstraint implements TrajectoryConstraint {

  private double minAcceleration;
  private double maxAcceleration;

  /**
   * Adds minimum and maximum acceleration confines, ideal to help the robot quickly speed up, but slowly slow down when reaching final destination
   * @param minAcceleration - should be negative
   * @param maxAcceleration - should be positive
   */
  public AccelerationConstraint(
    double minAcceleration,
    double maxAcceleration
  ) {
    this.minAcceleration = minAcceleration;
    this.maxAcceleration = maxAcceleration;
  }

  @Override
  public double getMaxVelocityMetersPerSecond(
    Pose2d poseMeters,
    double curvatureRadPerMeter,
    double velocityMetersPerSecond
  ) {
    return Double.MAX_VALUE;
  }

  /**
   * This is the only function overwritten here,
   * we return a minmax on the acceleration (putting an upper and lower bound, with what we are given)
   * Basically any trajectory made with this config must be bounded in this acceleration
   * I.e. trajectories are list of timesteps, i.e. at time T, the robot should be at x,y,z,theta
   * and we should expect to move Vx, Vy, Vz, Vtheta to get to the next time step
   * to get to the Vx, Vy, Vz, there are acceleration variables as well
   * This constraint limits the acceleration variables
   * This is easier to visualize if you look at a pathweaver trajectory json file!!!!
   */
  @Override
  public MinMax getMinMaxAccelerationMetersPerSecondSq(
    Pose2d poseMeters,
    double curvatureRadPerMeter,
    double velocityMetersPerSecond
  ) {
    return new MinMax(minAcceleration, maxAcceleration);
  }
}
