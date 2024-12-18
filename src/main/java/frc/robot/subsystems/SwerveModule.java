// TODO BSM - Add a File comment header.  Post Season let's update to remove deprecated imports.
/* TODO NEXT YEAR
 * This is a new coder thing to do! Update the swerve code hehe
 * Otherwise this is from some other source that I don't remember
 * This represents one corner of a swerve robot though
 */
package frc.robot.subsystems;
// Phoenix 
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
// Rev
import com.revrobotics.AbsoluteEncoder;
//import com.revrobotics.CANSparkBase.IdleMode;
//import com.revrobotics.SparkMax;
//import com.revrobotics.SparkMax.IdleMode;
//import com.revrobotics.SparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
//import com.revrobotics.SparkMaxPIDController;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {

  private final SparkMax driveMotor;
  private final SparkMax turningMotor;
  private final SparkMaxConfig driveMotorCfg;
  private final SparkMaxConfig turningMotorCfg;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turningEncoder;

  private final CANcoder absoluteEncoder;
  private       CANcoderConfigurator canCoderConfig;
  private       CANcoderConfiguration canCoderConfiguration;
  private final boolean absoluteEncoderReversed;
  // Health of the Magneti Encoder
  private final StatusSignal<MagnetHealthValue> magnetHealth;
  /// Angle of absolute encoder
  private final StatusSignal<Angle> angle;

  private final PIDController turningPidController;

  // Range is in degrees 0 -360
  private final double absoluteEncoderOffset;
  // Current set state;
  SwerveModuleState m_state;

  public SwerveModule(
    int driveMotorId,
    int turningMotorId,
    boolean driveMotorReversed,
    boolean turningMotorReversed,
    int absoluteEncoderId,
    double absoluteEncoderOffset,
    boolean absoluteEncoderReversed
  ) {
    // Create absolute encoders
    this.absoluteEncoderOffset = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteEncoder = new CANcoder(absoluteEncoderId, "rio");
    canCoderConfiguration = new CANcoderConfiguration();
    magnetHealth = absoluteEncoder.getMagnetHealth();
    angle = absoluteEncoder.getAbsolutePosition();

    canCoderConfig = absoluteEncoder.getConfigurator();

    // See class CanCoderConfiguration for options.
    // Boot the can to absolute mode - This is set the flash by the tuner X, but make sure
    // not to change it here.
    // Also set the sensor direction.
    // Be default the sensorCoefficient is set for degrees (360.0 / 4096.0) and units - "Deg"
    // To change it to radians adjust these values.
    canCoderConfig.refresh(canCoderConfiguration.MagnetSensor);
    canCoderConfiguration.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(1));
    canCoderConfiguration.MagnetSensor.withSensorDirection(absoluteEncoderReversed ? SensorDirectionValue.Clockwise_Positive
                                                               : SensorDirectionValue.CounterClockwise_Positive);
    canCoderConfiguration.MagnetSensor.withMagnetOffset(absoluteEncoderOffset / 360);
    canCoderConfig.apply(canCoderConfiguration);

    // canCoderConfiguration.absoluteSensorRange =
    //   AbsoluteSensorRange.Unsigned_0_to_360;
    // canCoderConfiguration.initializationStrategy =
    //   SensorInitializationStrategy.BootToAbsolutePosition;
    // canCoderConfiguration.withSensorDirection(sensorDirection = absoluteEncoderReversed;
    // absoluteEncoder.configAllSettings(canCoderConfiguration);

    if (DebugConstants.enableSmartDash) {
      SmartDashboard.putString(
        getName() + "/abs/config",
        canCoderConfiguration.toString()
      );
    }

    // Create Motors
    driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
    turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
    driveMotorCfg = new SparkMaxConfig();
    turningMotorCfg = new SparkMaxConfig();

    // Motor inversion for the encoders
    driveMotorCfg.inverted(driveMotorReversed);
    turningMotorCfg.inverted(turningMotorReversed);

    driveMotorCfg.idleMode(IdleMode.kBrake);
    turningMotorCfg.idleMode(IdleMode.kBrake);

    // Get Motor Encoders
    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();

    driveMotorCfg.idleMode(IdleMode.kBrake);

    // Encoder conversion constants
    driveMotorCfg.encoder.positionConversionFactor(
      ModuleConstants.kDriveEncoderRot2Meter
    );
    driveMotorCfg.encoder.velocityConversionFactor(
      ModuleConstants.kDriveEncoderRPM2MeterPerSec
    );
    // Causes the get poition to return value in Radians
    turningMotorCfg.encoder.positionConversionFactor(
      ModuleConstants.kTurningEncoderRot2Rad
    );
    turningMotorCfg.encoder.velocityConversionFactor(
      ModuleConstants.kTurningEncoderRPM2RadPerSec
    );

    // Set spark configs
    driveMotor.configure(driveMotorCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turningMotor.configure(turningMotorCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Pid for turning - should use the embedded one.
    turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
    // Circular System, can loop
    turningPidController.enableContinuousInput(0, 2 * Math.PI);

    // Reset Encoders At Start
    resetEncoders();
  }

  @Override
  public void periodic() {
    if (DebugConstants.enableSmartDash) {
      // Absolute Encoder Values
      SmartDashboard.putNumber(
        getName() + "/abs/Id",
        absoluteEncoder.getDeviceID()
      );
      SmartDashboard.putNumber(
        getName() + "/abs/AbsPosDegrees",
        angle.getValue().in(Degrees)
      );
      // SmartDashboard.putNumber(
      //   getName() + "/abs/RelPos",
      //   angle.getValue().in()
      // );
      // SmartDashboard.putNumber(
      //   getName() + "/abs/BusVoltage",
      //   absoluteEncoder.getBusVoltage()
      // );
      SmartDashboard.putNumber(
        getName() + "/abs/Radians",
        getAbsoluteEncoderRad()
      );
      addChild(getName() + " absEncChild", absoluteEncoder);
      SmartDashboard.putData(absoluteEncoder);
      // Turning Motor / Encoder
      SmartDashboard.putNumber(
        getName() + "/Turn/PositionRad",
        turningEncoder.getPosition()
      );
      SmartDashboard.putNumber(
        getName() + "/Turn/VelRadPerSec",
        turningEncoder.getVelocity()
      );
      // Driving
      SmartDashboard.putNumber(
        getName() + "/Drive/Position",
        driveEncoder.getPosition()
      );

      SmartDashboard.putNumber(
        getName() + "/Drive/Velocity",
        driveEncoder.getVelocity()
      );
      // Desired state
      if (m_state != null) {
        SmartDashboard.putString(getName() + "/State", m_state.toString());
      }
    }
  }

  // Gets the drive position in meters due to the conversion factor
  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  // Gets the turning encoder position in radians
  public double getTurningPosition() {
    return turningEncoder.getPosition();
  }

  // Drive velocity in meters per second
  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  // Truning velocity in radians per second.
  public double getTurningVelocity() {
    return turningEncoder.getVelocity();
  }

  // Get the angle of the absolute encoder in radians.  Because the offset is removed
  // This is the angle relative to forward for each wheel.
  public double getAbsoluteEncoderRad() {
    // Note the absolute position is in degrees (0-360)
    double encoderPos = angle.getValue().in(Radians);
    return encoderPos; // * (absoluteEncoderReversed ? -1.0 : 1.0); // Multiply by -1 if reversed
  }

  public void resetEncoders() {
    // Zero Drive motor Encoders
    driveEncoder.setPosition(0);

    // Set turning encoder to absolute encoder value (same as wheel)
    // This caused them to track the same in radians.
    turningEncoder.setPosition(getAbsoluteEncoderRad());
  }

  // TODO - Return swervemodule position (?? no clue what this meant)
  public SwerveModulePosition getSwerveModulePosition() {
    return new SwerveModulePosition(getDrivePosition(), getState().angle);
  }

  // Allow to Get Info In SweveModulePosition Format
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      getDriveVelocity(),
      new Rotation2d(getAbsoluteEncoderRad())
    );
  }

  // Set the motors to a desired state, the state input is a speed and and angle.
  public void setDesiredState(SwerveModuleState state) {
    if (DebugConstants.enableSmartDash) {
      // Input State
      SmartDashboard.putString(getName() + "/InputState", state.toString());
    }
    // Check if no substantial drive velocity in new command, ignore command, stop motors
    // Fixes issue of wheel returning to 0 when release input
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }

    // Optimize Angle Set Point (move <90 deg)
    // This will cause it to adjust the angle to less that 90 and maybe reserve the speed.
    state = SwerveModuleState.optimize(state, getState().angle);

    // Save the state for internal use
    m_state = state;

    // Scale velocity based on max speed
    driveMotor.set(
      state.speedMetersPerSecond /
      DriveConstants.kPhysicalMaxSpeedMetersPerSecond
    );

    // Calculate output from PID
    turningMotor.set(
      turningPidController.calculate(
        getTurningPosition(),
        state.angle.getRadians()
      )
    );

    if (DebugConstants.enableSmartDash) {
      // Optimized State
      SmartDashboard.putString(getName() + "/OptState", state.toString());
    }
  }

  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }
}
