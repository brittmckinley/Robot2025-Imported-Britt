package frc.robot.commands.DriveTrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.RobotState;
import frc.robot.utils.RobotState.RobotLocations;
import java.util.Optional;
import java.util.function.Supplier;

public class RunSwerveTeleop extends Command {

  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final SlewRateLimiter xLimiter, yLimiter;
  private final Supplier<Integer> anglePOWFunction;

  private boolean sensitivityDownPressed = false;
  private boolean sensitivityUpPressed = false;
  private final Supplier<Boolean> sensitivityDownFunction;
  private final Supplier<Boolean> sensitivityUpFunction;
  private final Supplier<Boolean> sensitivityToggleFunction;

  private double sensitivity = 1;

  private PIDController angleDriftPidController;
  private Timer driveAngleTimer = new Timer();
  private double previousTime = 0;
  private double allianceOffset = 0;

  private boolean toggleSensitivityCurrentState = false;
  private boolean toggleSensitvityPreviousState = false;
  private boolean toggleSensitivity = true;

  public RunSwerveTeleop(
    SwerveSubsystem swerveSubsystem,
    Supplier<Double> xSpdFunction,
    Supplier<Double> ySpdFunction,
    Supplier<Double> turningSpdFunction,
    Supplier<Integer> anglePOWFunction,
    Supplier<Boolean> sensitivityDownFunction,
    Supplier<Boolean> sensitivityUpFunction,
    Supplier<Boolean> sensitivityToggleFunction
  ) {
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;

    // Set up the limiters to prevent over acceleration
    // Used to smoot out the drivkFieldOrientedBtning.
    this.xLimiter =
      new SlewRateLimiter(
        DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond
      );
    this.yLimiter =
      new SlewRateLimiter(
        DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond
      );

    this.anglePOWFunction = anglePOWFunction;

    this.sensitivityDownFunction = sensitivityDownFunction;
    this.sensitivityUpFunction = sensitivityUpFunction;
    this.sensitivityToggleFunction = sensitivityToggleFunction;

    this.angleDriftPidController = new PIDController(0.0825, 0, 0.000);
    this.angleDriftPidController.setTolerance(2.5);
    this.angleDriftPidController.enableContinuousInput(-180, 180);

    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    driveAngleTimer.reset();
    driveAngleTimer.start();

    swerveSubsystem.setExpectedAngleToHeading();
  }

  @Override
  public void execute() {
    // Get real-time joystick inputs
    double xSpeed = sensitivity * xSpdFunction.get();
    double ySpeed = sensitivity * ySpdFunction.get();
    double joystickTurningSpeed = turningSpdFunction.get();
    toggleSensitivityCurrentState = sensitivityToggleFunction.get();

    if (toggleSensitivityCurrentState) {
      if (toggleSensitvityPreviousState == false) {
        if (toggleSensitivity) {
          sensitivity = 0.35;
          toggleSensitivity = false;
        } else {
          sensitivity = 1;
          toggleSensitivity = true;
        }
      }
    }
    toggleSensitvityPreviousState = toggleSensitivityCurrentState;

    // Apply deadband -> if over deadband, then it forces 0-1 from the deadband limit to 1 (linear conversion from (deadband to 1) TO (0 to 1) )
    //This is to prevent the sudden jump when the xSpeed goes from 0 to deadband limit (0.1) -> smoother
    xSpeed =
      Math.abs(xSpeed) > OIConstants.kDeadband
        ? (xSpeed - Math.signum(xSpeed) * OIConstants.kDeadband) /
        (1 - OIConstants.kDeadband)
        : 0.0;
    ySpeed =
      Math.abs(ySpeed) > OIConstants.kDeadband
        ? (ySpeed - Math.signum(ySpeed) * OIConstants.kDeadband) /
        (1 - OIConstants.kDeadband)
        : 0.0;
    joystickTurningSpeed =
      Math.abs(joystickTurningSpeed) > OIConstants.kDeadband
        ? joystickTurningSpeed
        : 0.0;

    int presetAngle = anglePOWFunction.get(); //gets angle returned (45 for NE, etc.), -1 if not pressed

    // if (presetAngle != -1) {
    //   // Converts from 0-360 to -180 to 180
    //   // Want to preserve -1 incase of failure -> might want to change to > 360 if we ever want to set to -1?
    //   if (presetAngle != -1) {
    //     presetAngle = (int) Math.IEEEremainder(-presetAngle, 360);
    //   }

    //   // swaps -180 to 180 to have signs correct
    //   // This is a preference -> could be removed by adding some negatives around
    //   if (presetAngle == -180) {
    //     presetAngle = 180;
    //   }

    //   swerveSubsystem.setExpectedAngle(presetAngle);

    // }

    if (presetAngle != -1) {
      switch (presetAngle) {
        case 0:
          break;
        case 90:
          RobotState.setRobotLocation(RobotLocations.SPEAKER);
          break;
        case 180:
          //nada
          RobotState.setRobotLocation(RobotLocations.INTAKE);
          break;
        case 270:
          RobotState.setRobotLocation(RobotLocations.AMP);
          break;
        default:
          break;
      }
    }

    SmartDashboard.putBoolean(
      "ROBOT STATE INTAKE",
      RobotState.getRobotLocation() == RobotLocations.INTAKE
    );
    SmartDashboard.putBoolean(
      "ROBOT STATE SPEAKER",
      RobotState.getRobotLocation() == RobotLocations.SPEAKER
    );
    SmartDashboard.putBoolean(
      "ROBOT STATE AMP",
      RobotState.getRobotLocation() == RobotLocations.AMP
    );
    SmartDashboard.putBoolean(
      "ROBOT STATE UNKNOWN",
      RobotState.getRobotLocation() == RobotLocations.UNKNOWN
    );

    // Sensitivity Control
    if (sensitivityDownFunction.get()) {
      if (!sensitivityDownPressed) {
        sensitivityDownPressed = true;
        sensitivity = Math.min(Math.max(0, sensitivity - 0.05), 1); //Forces range of sensitivity between [0,1]
      }
    } else {
      if (sensitivityDownPressed) {
        sensitivityDownPressed = false;
      }
    }
    if (sensitivityUpFunction.get()) {
      if (!sensitivityUpPressed) {
        sensitivityUpPressed = true;
        sensitivity = Math.min(Math.max(0, sensitivity + 0.05), 1);
      }
    } else {
      if (sensitivityUpPressed) {
        sensitivityUpPressed = false;
      }
    }

    SmartDashboard.putNumber("sensitivity", sensitivity);

    // Make the driving smoother
    // Causes the speed to ramp up to a max acceleration
    xSpeed =
      xLimiter.calculate(xSpeed) *
      DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed =
      yLimiter.calculate(ySpeed) *
      DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    joystickTurningSpeed =
      joystickTurningSpeed *
      DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    //Get time taken to run this function again
    double currentTime = driveAngleTimer.get();
    double dTime = currentTime - previousTime;
    previousTime = currentTime;

    //Controls angle pid
    double dAngle = 0.8 * joystickTurningSpeed * dTime;
    swerveSubsystem.setExpectedYaw(
      swerveSubsystem.getExpectedYaw() + Math.toDegrees(dAngle)
    );

    double expectedAngleConstraint = Math.IEEEremainder(
      swerveSubsystem.getExpectedYaw(),
      360
    );

    double pidOutput = angleDriftPidController.calculate(
      swerveSubsystem.getHeading(),
      expectedAngleConstraint
    );
    SmartDashboard.putNumber("Swerve current pid output for angle", pidOutput);

    // Make PID weak: Subtract output from turningSpeed
    if (angleDriftPidController.atSetpoint()) {
      pidOutput = 0;
    }
    double finalTurnSpeed = MathUtil.clamp(
      pidOutput,
      -Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond,
      DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond
    );

    // Construct desired chassis speeds
    ChassisSpeeds chassisSpeeds;

    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (DriverStation.getAlliance().isPresent()) {
      if (alliance.get() == Alliance.Red) {
        allianceOffset = Math.PI;
      } else {
        allianceOffset = 0;
      }
    }

    chassisSpeeds =
      ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed, //change to change sides of robot
        ySpeed,
        finalTurnSpeed,
        swerveSubsystem.getRotation2d().plus(new Rotation2d(allianceOffset)) //flips 180 degrees if on red team (robot position always relative to blue side)
      );

    //Helps fix with translational drifting
    chassisSpeeds = SwerveSubsystem.discretize(chassisSpeeds, dTime); //20 ms

    if (DebugConstants.enableSmartDash) {
      SmartDashboard.putNumber(
        "chassis Speed X",
        chassisSpeeds.vxMetersPerSecond
      );
      SmartDashboard.putNumber(
        "chassis Speed Y",
        chassisSpeeds.vyMetersPerSecond
      );
      SmartDashboard.putNumber(
        "chassis Speed Theta",
        chassisSpeeds.omegaRadiansPerSecond
      );
    }

    // Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
      chassisSpeeds
    );

    // Output each module states to wheels
    swerveSubsystem.setModuleStates(moduleStates);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop Motors
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    // Keep Running Joystick Code
    return false;
  }
}
