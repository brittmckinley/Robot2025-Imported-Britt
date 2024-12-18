// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake.DEBUG;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class RunIntakeTilNote extends Command {

  private Intake m_intake;
  private Timer intakeCurrentTimer;
  private boolean stopIntake = false;

  /** Creates a new RunIntakeTilNote. */
  public RunIntakeTilNote(Intake m_intake) {
    this.intakeCurrentTimer = new Timer();
    this.m_intake = m_intake;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setSpinMotor(Constants.IntakeConstants.kIntakeSpeedIntake);
    intakeCurrentTimer.stop();
    intakeCurrentTimer.reset();
    intakeCurrentTimer.start();
    stopIntake = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //detects if we have a note in
    if (
      m_intake.getTheoreticalWheelCurrent() >
      Constants.IntakeConstants.kWheelOutputCurrentLimit
    ) {
      //detects high current for x milliseconds, if so we have a note
      if (
        intakeCurrentTimer.get() >
        Constants.IntakeConstants.kIntakeWheelCurrentTime
      ) {
        intakeCurrentTimer.stop();
        intakeCurrentTimer.reset();
        m_intake.resetNoteRegister(true);
        stopIntake = true;
      } else {
        intakeCurrentTimer.start();
      }
    } else {
      intakeCurrentTimer.stop();
      intakeCurrentTimer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setSpinMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stopIntake;
  }
}
