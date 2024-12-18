// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain.DEBUG;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveTrain.DriveToPosition;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToAprilTagShootArc extends SequentialCommandGroup {

  private SwerveSubsystem m_driveTrain;
  private Camera m_camera;

  private double radius = 5;
  private Pose2d aprilTagLoc;

  /** Creates a new DriveToAprilTagShootArc. */
  public DriveToAprilTagShootArc(
    SwerveSubsystem m_driveTrain,
    Camera m_camera
  ) {
    this.m_driveTrain = m_driveTrain;
    this.m_camera = m_camera;

    addCommands(new DriveToPosition(m_driveTrain, () -> getFinalPosition()));
  }

  public Pose2d getAprilTagPose2d(Camera m_camera) {
    Pose2d aprilTag = m_camera.getAprilTagLoc();
    return aprilTag;
  }

  public Pose2d getFinalPosition() {
    Pose2d aprilTagLoc = m_camera.getAprilTagLoc();
    double norm = aprilTagLoc.getTranslation().getNorm();
    Pose2d arcPos = aprilTagLoc.times((norm - radius) / norm);
    arcPos =
      new Pose2d(
        arcPos.getX(),
        arcPos.getY(),
        new Rotation2d(180 / Math.PI * Math.atan2(arcPos.getY(), arcPos.getX()))
      );
    return arcPos;
  }
}
