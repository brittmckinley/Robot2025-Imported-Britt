// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain.DEBUG;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.DriveTrain.DriveToPosition;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.Supplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToTagGroup extends SequentialCommandGroup {

  private SwerveSubsystem m_driveTrain;
  private Camera m_camera;

  private double aprilTagRotation = 0;
  private double currentHeading = 0;

  /** Creates a new DriveToTagGroup. */
  public DriveToTagGroup(
    SwerveSubsystem m_driveTrain,
    Camera m_camera,
    Supplier<Double> distanceFromAprilTag,
    Supplier<Double> perpendicularDistanceFromAprilTag
  ) {
    this.m_driveTrain = m_driveTrain;
    this.m_camera = m_camera;

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
    currentHeading = Math.toRadians(m_driveTrain.getHeading());

    addCommands(
      new DriveToPosition(
        m_driveTrain,
        () ->
          getFinalPosition(
            distanceFromAprilTag.get(),
            perpendicularDistanceFromAprilTag.get()
          )
      )
    );
  }

  public Pose2d getAprilTagPose2d(Camera m_camera) {
    Pose2d aprilTag = m_camera.getAprilTagLoc();
    return aprilTag;
  }

  /**
   * Creates a final positon of the robot relative to an april tag
   * @param m_driveTrain
   * @param m_camera
   * @param distanceFromAprilTag distance away from the april tag (1 meter away)
   * @param perpendicularDistanceFromAprilTag strafe distance (1 meter to the left or right), + is right, - is left
   * @return
   */
  public Pose2d getFinalPosition(
    double distanceFromAprilTag,
    double perpendicularDistanceFromAprilTag
  ) {
    double meterOffFromAprilTag =
      distanceFromAprilTag + Constants.DriveConstants.kWheelBase / 2; //adds offset from
    Pose2d taglocRobotRelative = getAprilTagPose2d(m_camera); //gets april tag relative to center of robot

    if (taglocRobotRelative == null) {
      return null;
    }

    double x = taglocRobotRelative.getRotation().getCos();
    double y = taglocRobotRelative.getRotation().getSin();
    double meterNormalOff = perpendicularDistanceFromAprilTag;

    aprilTagRotation =
      Math.IEEEremainder(
        -Math.toDegrees(
          -m_driveTrain.getRotation2d().getRadians() +
          taglocRobotRelative.getRotation().getRadians()
        ),
        360
      );

    Pose2d taglocOffsetRobotRelative = new Pose2d(
      taglocRobotRelative.getX() -
      x *
      meterOffFromAprilTag -
      y *
      meterNormalOff,
      -taglocRobotRelative.getY() +
      y *
      meterOffFromAprilTag +
      x *
      meterNormalOff,
      new Rotation2d(aprilTagRotation * Math.PI / 180)
    );

    return taglocOffsetRobotRelative;
  }
}
