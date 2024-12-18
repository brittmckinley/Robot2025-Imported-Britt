// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain.DEBUG;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DebugConstants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.Supplier;

public class RotateToAngle extends Command {

  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> targetAngleSupplier;
  private double targetAngle = 0;

  private double turningSpeed = 0;
  private Timer angleTimer = new Timer();
  private PIDController angPidController = new PIDController(0.02, 0, 0.00);

  private SlewRateLimiter turningLimiter;

  /***
   * Rotates the robot towards a certain angle globaly
   * @param swerveSubsystem
   * @param targetAngle (in degrees)
   */
  public RotateToAngle(
    SwerveSubsystem swerveSubsystem,
    Supplier<Double> targetAngle
  ) {
    this.swerveSubsystem = swerveSubsystem;
    this.targetAngleSupplier = targetAngle;
    this.addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.turningSpeed = 0;
    this.angleTimer.reset();
    this.targetAngle = targetAngleSupplier.get();
    this.angPidController.setTolerance(2.5);
    this.angPidController.enableContinuousInput(-180, 180);

    this.turningLimiter =
      new SlewRateLimiter(
        Constants.DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond
      );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentHeading = swerveSubsystem.getHeading();

    // temporary value as currentTarget must persist over time
    // (currentTarget is used to compare whether the user has decided to swap angles

    turningSpeed = angPidController.calculate(currentHeading, targetAngle);

    if (DebugConstants.enableSmartDash) {
      SmartDashboard.putNumber("currentHeading", currentHeading);
      SmartDashboard.putNumber("target", targetAngle);
      SmartDashboard.putNumber("speed", turningSpeed);

      SmartDashboard.putBoolean("AtSetPoint", angPidController.atSetpoint());
      SmartDashboard.putNumber("error", angPidController.getError());
      SmartDashboard.putNumber("Timer", angleTimer.get());
    }

    // Detects end of PID
    // When robot is within threshold of the target for 1.5 secondes, rotation is
    // finished
    if (Math.abs(angPidController.getError()) < 5) {
      // 1.5 second timer disables subcommand
      if (angleTimer.get() > 1.5) {
        //
      } else {
        angleTimer.start();
      }
    } else {
      // checks if timer is running, if so stop it
      if (angleTimer.get() > 0.1) {
        angleTimer.stop();
        angleTimer.reset();
      }
    }

    turningSpeed =
      turningLimiter.calculate(turningSpeed) *
      Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    ChassisSpeeds chassisSpeeds;

    chassisSpeeds = new ChassisSpeeds(0, 0, turningSpeed);

    // 5. Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(
      chassisSpeeds
    );

    // 6. Output each module states to wheels
    swerveSubsystem.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean withinTargetError =
      Math.abs(angPidController.getError()) < 5;
    boolean angleTimerTime = angleTimer.get() > 0.2;
    return (withinTargetError && angleTimerTime);
  }
}
