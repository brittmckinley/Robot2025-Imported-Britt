// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import java.util.function.Supplier;

public class RunIntakeTeleop extends Command {

  private Intake m_intake;

  private Timer intakeCurrentTimer;

  private Timer atFlipPosition = new Timer();
  private boolean runningFlipPID = false;

  private Supplier<Boolean> flipIntake;
  private boolean flipIntakePreviousState = false;
  private Supplier<Boolean> pulseIntakeWheels;
  private boolean runningIntake = false;

  private boolean deployed = false;

  /** Creates a new RunIntakeTeleop. */
  public RunIntakeTeleop(
    Intake m_intake,
    Supplier<Boolean> flipIntake,
    Supplier<Boolean> pulseIntakeWheels
  ) {
    this.m_intake = m_intake;
    this.flipIntake = flipIntake;
    this.pulseIntakeWheels = pulseIntakeWheels;
    this.intakeCurrentTimer = new Timer();
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeCurrentTimer.stop();
    intakeCurrentTimer.reset();
    atFlipPosition.stop();
    atFlipPosition.reset();
  }

  private boolean toggleSensitivity = false;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean flipIntakeCurrentState = flipIntake.get();
    boolean toggleIntakeWheelsCurrentState = pulseIntakeWheels.get();

    SmartDashboard.putBoolean(
      "intake currentFlipState",
      flipIntakeCurrentState
    );
    SmartDashboard.putBoolean("intake prevFlipState", flipIntakePreviousState);

    //use joystick inputs

    //gets if flip intake button pressed
    if (flipIntakeCurrentState) {
      if (flipIntakePreviousState == false) {
        //flip the intake on press
        runningFlipPID = true;
        if (deployed) {
          m_intake.handshakeIntake();
          m_intake.setSpinMotor(0);
          runningIntake = false;
        } else {
          m_intake.deployIntake();
          runningIntake = true;
        }
      }
    }

    //toggles the intake wheels

    //reponse to joystick
    //Flip response
    if (runningFlipPID) {
      double flipPidOutput = m_intake.getFlipPidOutput();
      SmartDashboard.putNumber("intake Pid Output", flipPidOutput);
      m_intake.setFlipMotor(0.9 * flipPidOutput);

      if (m_intake.atFlipPosition()) {
        atFlipPosition.start();
        if (atFlipPosition.get() > Constants.IntakeConstants.kFlipPIDTimeout) {
          runningFlipPID = false;
          atFlipPosition.stop();
          atFlipPosition.reset();
          m_intake.setFlipMotor(0);
        }
      } else {
        atFlipPosition.stop();
        atFlipPosition.reset();
      }
    }

    SmartDashboard.putBoolean("Running flip pid", runningFlipPID);

    //if we are moving the intake wheels
    if (runningIntake) {
      if (m_intake.isDeployed()) {
        if (m_intake.hasNote()) {
          m_intake.setSpinMotor(-Constants.IntakeConstants.kIntakeSpeedFeeding);
        } else {
          m_intake.setSpinMotor(Constants.IntakeConstants.kIntakeSpeedIntake);
        }
      } else if (m_intake.isInHandShake()) {
        m_intake.setSpinMotor(0); //FOR DEBUG/RESETTING PURPOSES
      }

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
          runningIntake = false;
          m_intake.resetNoteRegister(true);

          //gets if intake is at handshake, if so automatically fold
        } else {
          intakeCurrentTimer.start();
          // m_intake.resetNoteRegister(false);
        }
      } else {
        intakeCurrentTimer.stop();
        intakeCurrentTimer.reset();
      }
    } else {
      m_intake.setSpinMotor(0);
    }

    if (m_intake.isDeployed()) {
      deployed = true;
    } else {
      deployed = false;
    }

    flipIntakePreviousState = flipIntakeCurrentState;
    // toggleIntakeWheelsPreviousState = toggleIntakeWheelsCurrentState;
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.setFlipMotor(0);
    m_intake.setSpinMotor(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
