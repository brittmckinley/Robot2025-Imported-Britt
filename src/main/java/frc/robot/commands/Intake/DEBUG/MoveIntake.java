// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake.DEBUG;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DebugConstants;
import frc.robot.subsystems.Intake;

public class MoveIntake extends Command {

  private Intake m_intake;
  private double targetAngle;
  private Timer atPositionTimer;

  /** Creates a new DeployIntake. */
  public MoveIntake(Intake m_intake, double targetAngle) {
    this.m_intake = m_intake;
    this.targetAngle = targetAngle;
    this.atPositionTimer = new Timer();

    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setTargetAngle(targetAngle);
    atPositionTimer.reset();
    atPositionTimer.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pidOut = m_intake.getFlipPidOutput();
    m_intake.setFlipMotor(pidOut);

    if (DebugConstants.enableSmartDash) {
      SmartDashboard.putNumber("current pidOut", pidOut);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setFlipMotor(0);
    atPositionTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_intake.atFlipPosition()) {
      atPositionTimer.start();
      if (atPositionTimer.get() > Constants.IntakeConstants.kFlipPIDTimeout) {
        return true;
      }
    } else {
      atPositionTimer.stop();
      atPositionTimer.reset();
    }
    return false;
  }
}
