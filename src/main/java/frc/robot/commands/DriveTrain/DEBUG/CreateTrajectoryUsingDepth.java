// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain.DEBUG;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.Supplier;

public class CreateTrajectoryUsingDepth extends Command {

  private Camera m_camera;
  private SwerveSubsystem m_swerveSubsystem;
  private Pose2d startPos;
  private Pose2d endPos;
  private Supplier<Pose2d> endPosSupplier;

  /** Creates a new CreateTrajectoryUsingDepth. */
  public CreateTrajectoryUsingDepth(
    Camera m_camera,
    SwerveSubsystem m_swerveSubsystem,
    Supplier<Pose2d> endPoseSupplier
  ) {
    this.m_camera = m_camera;
    this.m_swerveSubsystem = m_swerveSubsystem;
    this.endPosSupplier = endPoseSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting CreateTraj");
    startPos = m_swerveSubsystem.getPose();
    endPos = endPosSupplier.get();
    m_camera.getDepthCameraData(endPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Trajectory traj = m_swerveSubsystem.generateTrajectoryUsingMidPoints(
      startPos,
      endPos,
      m_camera.getDepthTrajectoryRaw(),
      0
    );
    m_swerveSubsystem.setField(traj);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_camera.getDepthTrajectoryRequestFulfilled();
  }
}
