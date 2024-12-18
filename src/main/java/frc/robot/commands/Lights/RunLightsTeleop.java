// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lights;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.SwerveSubsystem;

public class RunLightsTeleop extends Command {

  private Lights m_lights;
  private Intake m_intake;
  private SwerveSubsystem m_swerveSubsystem;

  public RunLightsTeleop(
    Lights m_lights,
    Intake m_intake,
    SwerveSubsystem m_swerveSubsystem
  ) {
    this.m_lights = m_lights;
    this.m_intake = m_intake;
    this.m_swerveSubsystem = m_swerveSubsystem;

    addRequirements(m_lights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.m_lights.setIdleMode(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_intake.isDeployed()) {
      m_lights.setStatusLedsIntakeDeployed();
      m_lights.resetStatusLedsOffset();
    } else if (m_intake.hasNote()) {
      m_lights.setStatusLedsIntakeHasNote();
    } else {
      m_lights.setAllianceRippleEffectUpr();
      m_lights.resetStatusLedsOffset();
    }

    m_lights.setAllianceRippleEffectUnder();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.m_lights.setIdleMode(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
