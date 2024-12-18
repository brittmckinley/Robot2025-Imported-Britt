// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ModifiedWPILibCommands.DynamicSwerveControllerAtInitWithRotateCommand;
import frc.robot.commands.ModifiedWPILibCommands.WaitCommandDynamic;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.List;
import java.util.function.Supplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToPositionAndRotate extends SequentialCommandGroup {

  private SwerveSubsystem m_driveTrain;
  private double currentHeading = 0;
  private TrajectoryConfig config = Constants.DriveConstants.kTrajectoryConfig;
  private Supplier<List<Pose2d>> intermediatePoses = () -> null;
  private Supplier<Rotation2d> rotationSupplier;
  private double minimumTimeNeeded = 0;
  private Trajectory currentTrajectory = null;
  private double delay = 0;
  private double neededTime = 0;

  // Constructor #1
  public DriveToPositionAndRotate(
    SwerveSubsystem m_driveTrain,
    Supplier<Pose2d> finalLoc,
    TrajectoryConfig config,
    Supplier<Rotation2d> rotationSupplier,
    double timeNeeded
  ) {
    this(m_driveTrain, finalLoc, rotationSupplier, timeNeeded);
    this.config = config;
  }

  // Constructor #2
  public DriveToPositionAndRotate(
    SwerveSubsystem m_driveTrain,
    Supplier<Pose2d> finalLoc,
    TrajectoryConfig config,
    Supplier<List<Pose2d>> intermediatePoses,
    Supplier<Rotation2d> rotationSupplier,
    double timeNeeded
  ) {
    this(m_driveTrain, finalLoc, rotationSupplier, timeNeeded);
    this.config = config;
    this.intermediatePoses = intermediatePoses;
  }

  // Constructor #2
  /***
   * Moves the robot to a set location
   * @param m_driveTrain
   * @param m_camera
   * @param finalLoc -> final position of the robot (relative to center of robot)
   * @param finalRotation -> rotation of the robot in degrees, robot will rotate at the start
   */
  public DriveToPositionAndRotate(
    SwerveSubsystem m_driveTrain,
    Supplier<Pose2d> finalLoc,
    Supplier<Rotation2d> rotationSupplier,
    double timeNeeded
  ) {
    this.m_driveTrain = m_driveTrain;
    this.rotationSupplier = rotationSupplier;
    this.neededTime = timeNeeded;

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
        this.currentHeading = Math.toRadians(m_driveTrain.getHeading());
        currentTrajectory = generateTrajectory(finalLoc.get());
        double delayNeeded =
          neededTime - currentTrajectory.getTotalTimeSeconds();
        if (delayNeeded > 0) {
          delay = delayNeeded;
        } else {
          delay = 0;
        }
      }),
      new WaitCommandDynamic(() -> delay),
      new DynamicSwerveControllerAtInitWithRotateCommand(
        () -> {
          Trajectory traj = generateTrajectory(finalLoc.get());
          m_driveTrain.prepareTrajectoryToRun(traj);
          return traj;
        },
        m_driveTrain::getPose,
        DriveConstants.kDriveKinematics,
        xController,
        yController,
        thetaController,
        m_driveTrain::setModuleStates,
        rotationSupplier,
        m_driveTrain
      )
      // new RotateToAngle(m_driveTrain, () -> finalLoc.get().getRotation().getDegrees())
    );
  }

  private Trajectory generateTrajectory(Pose2d finalPosition) {
    Pose2d initialLocation = m_driveTrain.getPose();

    Pose2d finalPositionRelativeToField = finalPosition;
    Rotation2d deltaRotInit = finalPosition
      .getTranslation()
      .minus(initialLocation.getTranslation())
      .getAngle();

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
