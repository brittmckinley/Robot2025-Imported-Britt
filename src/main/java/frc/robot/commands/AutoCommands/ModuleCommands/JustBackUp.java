// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.ModuleCommands;

/* Descrition:
 * Runs a parallel command group tp drive to the provided position.
 * While doing that also keep the shooter angle and run the camera to
 * maintain location data.
 */
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Camera.CameraPositionSetter;
import frc.robot.commands.DriveTrain.DriveToPosition;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.MathExtendedUtil;

public class JustBackUp extends ParallelCommandGroup {

  /***
   * Backs the robot out to a position
   * @param swerveSubsystem
   * @param m_intake
   * @param m_shooter
   * @param m_camera
   * @param backOutPosition - back out position
   */
  public JustBackUp(
    SwerveSubsystem swerveSubsystem,
    Intake m_intake,
    Camera m_camera,
    Pose2d backOutPosition
  ) {
    addCommands(
      new DriveToPosition(
        swerveSubsystem,
        () -> MathExtendedUtil.getAlliedFieldPosition(backOutPosition)
      ),
      new CameraPositionSetter(m_camera, swerveSubsystem)
    );
  }
}
