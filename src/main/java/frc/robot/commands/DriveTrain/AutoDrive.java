// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.MathExtendedUtil;
import frc.robot.utils.RobotState;
import java.util.ArrayList;
import java.util.function.Supplier;

public class AutoDrive extends SequentialCommandGroup {

  /***
   * Aligns the robot to certain locations
   * Can be override with joystick
   * @param m_swerve
   * @param m_camera
   * @param xSpdFunction - used to detect if the driver wants to stop auto drive
   * @param ySpdFunction - ^^
   * @param turningSpdFunction - ^^
   */
  public AutoDrive(
    SwerveSubsystem m_swerve,
    Camera m_camera,
    Supplier<Double> xSpdFunction,
    Supplier<Double> ySpdFunction,
    Supplier<Double> turningSpdFunction
  ) {
    addCommands(
      new DriveToPosition(
        m_swerve,
        () -> {
          switch (RobotState.getRobotLocation()) {
            case AMP:
              return MathExtendedUtil.getAlliedFieldPosition(
                Constants.LocationConstants.blueAmpLocation
              );
            case SPEAKER:
              return calculateShortestPoseToSpeaker(m_swerve);
            case INTAKE:
              return MathExtendedUtil.getAlliedFieldPosition(
                Constants.LocationConstants.blueIntakeLocation
              );
            default:
              return m_swerve.getPose();
          }
        },
        Constants.DriveConstants.kTrajectoryConfigSuperSlow,
        () -> {
          ArrayList<Pose2d> returnList = new ArrayList<>();

          switch (RobotState.getRobotLocation()) {
            case AMP:
              Pose2d ampLoc = MathExtendedUtil.getAlliedFieldPosition(
                Constants.LocationConstants.blueAmpLocation
              );
              ampLoc =
                new Pose2d(
                  ampLoc.getX(),
                  ampLoc.getY() - 0.6,
                  ampLoc.getRotation()
                );
              returnList.add(ampLoc);
              break;
            case SPEAKER:
              break;
            case INTAKE:
              break;
            default:
              break;
          }

          return returnList;
        }
      )
        .raceWith(
          new WaitUntilCommand(() -> {
            boolean action =
              !MathUtil.isNear(xSpdFunction.get(), 0, 0.15) ||
              !MathUtil.isNear(ySpdFunction.get(), 0, 0.15) ||
              !MathUtil.isNear(turningSpdFunction.get(), 0, 0.15);
            return action;
          })
        )
    );
  }

  //Self explanatory
  //Creates a vector from speaker center to current robot, finds its norm then multiples by radius.
  //This gives a vector that is added to the center of the speaker, which gives us the robot location on the arc around the robot (which is the shortest distance if we could go in a straight line)
  public Pose2d calculateShortestPoseToSpeaker(SwerveSubsystem m_swerve) {
    Pose2d currentPose = m_swerve.getPose();
    Pose2d speakerPosition = MathExtendedUtil.getAlliedFieldPosition(
      Constants.LocationConstants.blueSpeakerCenter
    );

    double deltaX = -speakerPosition.getX() + currentPose.getX();
    double deltaY = -speakerPosition.getY() + currentPose.getY();
    double deltaDistance = Math.pow((deltaX * deltaX + deltaY * deltaY), 0.5);
    deltaX *=
      Constants.LocationConstants.blueSpeakerRadiusMeters / deltaDistance;
    deltaY *=
      Constants.LocationConstants.blueSpeakerRadiusMeters / deltaDistance;

    return new Pose2d(
      speakerPosition.getX() + deltaX,
      speakerPosition.getY() + deltaY,
      new Rotation2d(Math.atan2(-deltaY, -deltaX))
    );
  }
}
