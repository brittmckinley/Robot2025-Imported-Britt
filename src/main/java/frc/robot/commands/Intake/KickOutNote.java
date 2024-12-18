// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class KickOutNote extends Command {

  private Intake m_intake;
  private Timer timer;

  /** Creates a new KickOutNote. */
  public KickOutNote(Intake m_intake) {
    this.m_intake = m_intake;
    this.timer = new Timer();
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.deployIntake();
    timer.stop();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intake.isDeployed()) {
      timer.start();
      m_intake.setFlipMotor(0);
      m_intake.setSpinMotor(-Constants.IntakeConstants.kIntakeSpeedIntake);
    } else {
      double pidOut = m_intake.getFlipPidOutput();
      m_intake.setFlipMotor(pidOut);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setSpinMotor(Constants.IntakeConstants.kIntakeSpeedIntake);
    m_intake.setTargetAngle(Constants.IntakeConstants.kAngleHandshake);
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 0.4;
  }
}
