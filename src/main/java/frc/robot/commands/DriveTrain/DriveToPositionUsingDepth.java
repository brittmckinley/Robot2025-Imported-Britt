// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveTrain.DEBUG.RotateToAngle;
import frc.robot.commands.ModifiedWPILibCommands.DynamicSwerveControllerAtExecuteCommand;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.Consumer;
import java.util.function.Supplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToPositionUsingDepth extends SequentialCommandGroup {

  private SwerveSubsystem m_swerveSubsystem;
  private Camera m_camera;
  private double currentHeading = 0;
  private Supplier<Pose2d> endPosSupplier;

  /***
   * Moves the robot to a set location
   * @param m_driveTrain
   * @param m_camera
   * @param finalLoc -> final position of the robot (relative to center of robot)
   * @param finalRotation -> rotation of the robot in degrees, robot will rotate at the start
   */
  public DriveToPositionUsingDepth(
    SwerveSubsystem m_driveTrain,
    Camera m_camera,
    Supplier<Pose2d> finalLoc
  ) {
    this.m_swerveSubsystem = m_driveTrain;
    this.m_camera = m_camera;
    this.endPosSupplier = endPosSupplier;

    PIDController xController = new PIDController(
      1 * AutoConstants.kPXController,
      0,
      0
    );
    PIDController yController = new PIDController(
      1 * AutoConstants.kPYController,
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
    System.out.println("At drive to position: ");
    System.out.println(finalLoc.get().toString());
    addCommands(
      //psuedo initalization function (gets current heading at run time, not at robot startup)
      new InstantCommand(() -> {
        Pose2d intialPos = m_driveTrain.getPose();
        Rotation2d angleToGo = finalLoc.get().minus(intialPos).getRotation();
        intialPos = new Pose2d(intialPos.getX(), intialPos.getY(), angleToGo);
        this.currentHeading =
          Math.toRadians(intialPos.getRotation().getDegrees());

        m_camera.getDepthCameraData(finalLoc.get());
        cameraRequestSent = true;
        recalcTime.reset();
        recalcTime.start();
      }),
      new WaitUntilCommand(() -> {
        return generateTrajectory(finalLoc.get()) != null;
      }),
      new DynamicSwerveControllerAtExecuteCommand(
        () -> {
          generateTrajectory(finalLoc.get());
          return recentlyCreatedTraj;
        },
        m_driveTrain::getPose,
        DriveConstants.kDriveKinematics,
        xController,
        yController,
        thetaController,
        m_driveTrain::setModuleStates,
        () -> new Rotation2d(currentHeading),
        this::setPreviousVelocity,
        m_driveTrain
      ),
      new RotateToAngle(
        m_driveTrain,
        () -> finalLoc.get().getRotation().getDegrees()
      ),
      new InstantCommand(() -> {
        recentlyCreatedTraj = null;
      })
    );
  }

  private Trajectory recentlyCreatedTraj = null;
  private boolean cameraRequestSent = false;
  private double previousVelocity = 0;

  private void setPreviousVelocity(double vel) {
    previousVelocity = vel;
  }

  private Timer recalcTime = new Timer();

  private Trajectory generateTrajectory(Pose2d finalPosition) {
    if (cameraRequestSent == false) {
      m_camera.getDepthCameraData(finalPosition);
      cameraRequestSent = true;
    } else {
      if (m_camera.getDepthTrajectoryRequestFulfilled() == true) {
        cameraRequestSent = false;
        Trajectory newTraj = m_swerveSubsystem.generateTrajectoryUsingMidPoints(
          m_swerveSubsystem.getPose(),
          finalPosition,
          m_camera.getDepthTrajectoryRaw(),
          previousVelocity
        );

        if (newTraj != null) {
          recentlyCreatedTraj = newTraj;
          m_swerveSubsystem.setField(recentlyCreatedTraj);
        }
        m_camera.resetDepthCameraRequest();
      }
    }
    return recentlyCreatedTraj;
  }
}
