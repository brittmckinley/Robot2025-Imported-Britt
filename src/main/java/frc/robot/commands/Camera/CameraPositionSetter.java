// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Camera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.MathExtendedUtil;

public class CameraPositionSetter extends Command {

  private final Camera m_camera;
  private final SwerveSubsystem m_swerveSubsystem;
  private final Timer detectionTimer;

  /**
   * Used to get position data of robot from april tags using limelight
   * In other terms, uses april tag to set location of robot on field (some data processing here too)
   * @param m_camera
   * @param m_swerveSubsystem
   */
  public CameraPositionSetter(
    Camera m_camera,
    SwerveSubsystem m_swerveSubsystem
  ) {
    this.m_swerveSubsystem = m_swerveSubsystem;
    this.m_camera = m_camera;
    this.detectionTimer = new Timer();
    addRequirements(m_camera);
  }

  @Override
  public void initialize() {}

  private boolean firstTimeDetect = false; //used to keep track if an april tag has been detected

  @Override
  public void execute() {
    Pose2d estimatedVisonPosition = m_camera.getRobotsFieldPosition();

    //cheap way of detecting if april tag is detected or not
    if (
      estimatedVisonPosition.getX() != 0 || estimatedVisonPosition.getY() != 0
    ) {
      //further legitamacy test
      if (
        MathExtendedUtil.isPoseOnField(estimatedVisonPosition) &&
        m_swerveSubsystem.getWheelSpeed() < 0.5
      ) {
        detectionTimer.start();
        //removes random split second detections off of enviromental patterns (if we are certain the april tag is there, add it)
        if (detectionTimer.get() > Constants.CameraConstants.kDetectionTime) {
          m_swerveSubsystem.addVisionMeasurement(
            estimatedVisonPosition,
            m_camera.getLatency()
          );
          if (firstTimeDetect) {
            //resets gyro to be field oriented
            m_swerveSubsystem.setGyro(
              estimatedVisonPosition.getRotation().getDegrees()
            );
            firstTimeDetect = false;
          }
        } else {
          firstTimeDetect = true;
        }
      } else {
        resetTimer();
      }
    } else {
      resetTimer();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  //resets detection timer
  public void resetTimer() {
    detectionTimer.stop();
    detectionTimer.reset();
  }
}
