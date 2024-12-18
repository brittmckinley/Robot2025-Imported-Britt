// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveTrain.DEBUG.RotateToAngle;
import frc.robot.commands.ModifiedWPILibCommands.DynamicSwerveControllerAtInitCommand;
import frc.robot.commands.ModifiedWPILibCommands.DynamicSwerveControllerAtInitWithRotateCommand;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.List;
import java.util.function.Supplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToPosition extends SequentialCommandGroup {

  private SwerveSubsystem m_driveTrain;
  private double currentHeading = 0;
  private TrajectoryConfig config = Constants.DriveConstants.kTrajectoryConfig;
  private Supplier<List<Pose2d>> intermediatePoses = () -> null;
  private Rotation2d angleToRotate;

  public DriveToPosition(
    SwerveSubsystem m_driveTrain,
    Supplier<Pose2d> finalLoc,
    TrajectoryConfig config
  ) {
    this(m_driveTrain, finalLoc);
    this.config = config;
  }

  public DriveToPosition(
    SwerveSubsystem m_driveTrain,
    Supplier<Pose2d> finalLoc,
    TrajectoryConfig config,
    Supplier<List<Pose2d>> intermediatePoses
  ) {
    this(m_driveTrain, finalLoc);
    this.config = config;
    this.intermediatePoses = intermediatePoses;
  }

  /***
   * Moves the robot to a set location
   * @param m_driveTrain
   * @param m_camera
   * @param finalLoc -> final position of the robot (relative to center of robot)
   * @param finalRotation -> rotation of the robot in degrees, robot will rotate at the start
   */
  public DriveToPosition(
    SwerveSubsystem m_driveTrain,
    Supplier<Pose2d> finalLoc
  ) {
    this.m_driveTrain = m_driveTrain;

    PIDController xController = new PIDController(
      AutoConstants.kPXController,
      0,
      0
    );
    PIDController yController = new PIDController(
      AutoConstants.kPYController,
      0,
      0
    );

    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController,
      0,
      0,
      AutoConstants.kThetaControllerConstraints
    );

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    addCommands(
      //psuedo initalization function (gets current heading at run time, not at robot startup)
      new InstantCommand(() -> {
        //  angleToRotate = finalLoc.get().getTranslation().minus(m_driveTrain.getPose().getTranslation()).getAngle();
        angleToRotate = finalLoc.get().getRotation();
      }),
      // This is a modified WPILIB command.  See WPILIB desription
      // Drive following a trajectory.
      new DynamicSwerveControllerAtInitWithRotateCommand(
        () -> {
          Trajectory trajectory = generateTrajectory(finalLoc.get());
          m_driveTrain.prepareTrajectoryToRun(trajectory);
          return trajectory;
        },
        m_driveTrain::getPose,
        DriveConstants.kDriveKinematics,
        xController,
        yController,
        thetaController,
        m_driveTrain::setModuleStates,
        () -> angleToRotate,
        m_driveTrain
      )
      // new RotateToAngle(m_driveTrain, () -> finalLoc.get().getRotation().getDegrees())
    );
  }

  /**
   * Generates a trajectory dynamically (at the time the command is actually run instead of at the creation of the class at the start of the game)
   * Basically this creates a polygon path (path of straight lines between points)
   * The points we are drawing lines to are 1. current robot position -> 2 to (n-1). intermediatePoses, n. intented final position
   * See details of how this is done in the generateTrajectoryWithDiscreteIntermediates
   *
   * Example:
   *
   *     /-2-\       4
   *  1-/     \     /
   *           \-3-/
   * @param finalPosition
   * @return
   */
  private Trajectory generateTrajectory(Pose2d finalPosition) {
    Pose2d initialLocation = m_driveTrain.getPose();
    Pose2d finalPositionRelativeToField = finalPosition;

    // Rotation2d deltaRotInit = finalPosition
    //   .getTranslation()
    //   .minus(initialLocation.getTranslation())
    //   .getAngle(); no clue what this is?

    Trajectory trajectory = null;
    try {
      trajectory =
        m_driveTrain.generateTrajectoryWithDiscreteIntermediates(
          initialLocation,
          finalPositionRelativeToField,
          config,
          intermediatePoses.get()
        );
    } catch (Exception e) {
      System.out.println(e);
    }

    return trajectory;
  }
}
