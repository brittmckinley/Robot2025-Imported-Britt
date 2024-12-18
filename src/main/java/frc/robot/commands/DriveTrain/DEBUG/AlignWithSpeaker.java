// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain.DEBUG;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.MathExtendedUtil;

public class AlignWithSpeaker extends Command {

  private SwerveSubsystem m_swerveSubsystem;

  /** Creates a new AlignWithSpeaker. */
  public AlignWithSpeaker(SwerveSubsystem m_swerveSubsystem) {
    this.m_swerveSubsystem = m_swerveSubsystem;
    addRequirements(m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d robotPos = this.m_swerveSubsystem.getPose();
    Pose2d speakerPos = MathExtendedUtil.getAlliedFieldPosition(
      Constants.LocationConstants.blueSpeakerCenter
    );
    m_swerveSubsystem.setExpectedYaw(
      robotPos
        .getTranslation()
        .minus(speakerPos.getTranslation())
        .getAngle()
        .getDegrees()
    );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
