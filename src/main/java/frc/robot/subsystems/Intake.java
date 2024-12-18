// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

//import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.SparkMaxLowLevel.MotorType;
//import com.revrobotics.CANSparkLowLevel.MotorType;
//import com.revrobotics.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private final SparkMax intakeFlipMotor;
  private final SparkMax intakeWheelsMotor;
  SparkMaxConfig intakeFlipConfig;
  SparkMaxConfig intakeWheelsConfig;

  private final PIDController flipPidController;

  // abs encoder
  private DutyCycleEncoder absoluteEncoder;

  private double targetAngle = 0; //degrees

  private double angleDeployed = Constants.IntakeConstants.kAngleDeployed;
  private double angleHandshake = Constants.IntakeConstants.kAngleHandshake;
  private double angleDeployedMOE = 20; //EOM = margin of error (degrees)
  private double angleHandshakeMOE = 2;

  private boolean hasNote = false;

  // Position Limit Switches
  private final DigitalInput upperLimitSwitch;
  private final DigitalInput lowerLimitSwitch;

  public Intake() {
    intakeFlipMotor =
      new SparkMax(IntakeConstants.kFlipMotorID, MotorType.kBrushless);
    intakeFlipConfig = new SparkMaxConfig();
    intakeFlipConfig.idleMode(IdleMode.kBrake);
        // TODO:  BSM - Move to constants?   Where did the .04 come from? -> yea this was experimental
    //smooths robot arm by limiting time to get to max speed (0.4 seconds)
    intakeFlipConfig.openLoopRampRate(0.4);
    intakeFlipMotor.configure(intakeFlipConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    intakeWheelsMotor =
      new SparkMax(IntakeConstants.kWheelMotorID, MotorType.kBrushless);
    intakeWheelsConfig = new SparkMaxConfig();
    intakeWheelsConfig.idleMode(IdleMode.kBrake);
    intakeWheelsMotor.configure(intakeFlipConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    // Encoder object created to display position values
    // TODO BSM DutyCycle Encoders are new to me, reference doc or explain.
    // TODO BSM is there a constant for the angle reference? -> probably could reset in phoenix tuner/rev hardware
    // ANGLE REFERENCES WERE USED IN CONSTANTS THOUGH (angles for deployed and angles for inside the robot)
    // I do remember setting 0 to be horizontal with the floor though
    //https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/encoders-software.html#duty-cycle-encoders-the-dutycycleencoder-class
    absoluteEncoder = new DutyCycleEncoder(IntakeConstants.kAbsoluteEncoderID);

    // Setup PID
    flipPidController = new PIDController(0.01, 0, 0);
    flipPidController.enableContinuousInput(0, 360);

    // Setup Limit Switches
    upperLimitSwitch =
      new DigitalInput(IntakeConstants.kUpperLimitSwitchChannel);
    lowerLimitSwitch =
      new DigitalInput(IntakeConstants.kLowerLimitSwitchChannel);

  }

  @Override
  public void periodic() {
    if (DebugConstants.enableSmartDash) {
      SmartDashboard.putNumber("current Target angle", targetAngle);
      SmartDashboard.putNumber("Intake current Angle", getCurrentAngle());
      SmartDashboard.putBoolean(
        "Intake lower limit switch",
        lowerLimitSwitch.get()
      );
      SmartDashboard.putBoolean(
        "Intake upper limit switch",
        upperLimitSwitch.get()
      );
      SmartDashboard.putBoolean("Intake isDeployed", isDeployed());
      SmartDashboard.putNumber(
        "Intake Output current",
        getTheoreticalWheelCurrent()
      );
    }
  }

  public void deployIntake() {
    setTargetAngle(angleDeployed);
  }

  public void handshakeIntake() {
    setTargetAngle(angleHandshake);
  }

  public boolean isDeployed() {
    // TDOD BSM - Describe what the isNear does or reference the source doc.
    return MathUtil.isNear(
      angleDeployed,
      getCurrentAngle(),
      angleDeployedMOE, // TDOD BSM MOE?
      0,
      360
    );
  }

  public boolean isInHandShake() {
    return MathUtil.isNear(
      angleHandshake,
      getCurrentAngle(),
      angleHandshakeMOE,
      0,
      360
    );
  }

  public void setTargetAngle(double angle) {
    angle = MathUtil.clamp(angle, angleDeployed, angleHandshake); // Equivalent of Math.min(Math.max(angle, angleDeployed), angleHandshake);
    targetAngle = angle;
  }

  public double getTargetAngle() {
    return targetAngle;
  }

  public double getCurrentAngle() {
    return absoluteEncoder.get() * 360;
  }

  public double getFlipPidOutput() {
    double feedforward =
      Constants.IntakeConstants.kFlipMotorKV * Math.cos(getCurrentAngle());
    return (
      flipPidController.calculate(getCurrentAngle(), targetAngle) + feedforward
    );
  }

  public boolean atFlipPosition() {
    return MathUtil.isNear(targetAngle, getCurrentAngle(), 5, 0, 360);
  }

  /**
   * Sets motor of the flip motor of intake (moving the whole intake into the robot or onto the field)
   * A lot of saftey is implemented here to prevent the intake from being broken by having the motor place too much pressure on the intake
   * @param speed
   */
  public void setFlipMotor(double speed) {
    if (getLowerLimitSwitch() || (getCurrentAngle() > 350 - angleDeployedMOE)) { //if the intake is detected to be deployed
      if (speed < 0) { //speed < 0 means the robot is still moving towards the floor, but we are at the limit switch
        intakeFlipMotor.set(0.1); //goes the other way/relieve the intake from placing pressure onto the intake hand with the floor
      } else {
        intakeFlipMotor.set(speed); //moving the intake back up, don't block this (retracting)
      }
    } else if (getUpperLimitSwitch()) { //intake is fully in the robot, any further going this direction would hurt the arm
      if (speed > 0) {
        // Stop at limit switch
        intakeFlipMotor.set(0); //don't move any further into the robot (prevents pressure)
      } else {
        intakeFlipMotor.set(speed); //don't restrict
      }
    } else {
      intakeFlipMotor.set(speed); //no limit switch hit, meaning we are somewhere in the middle, which means its fine to go anywhere!
    }
  }

  public void setSpinMotor(double speed) {
    intakeWheelsMotor.set(speed);
  }

  public double getWheelSpeed() {
    return intakeWheelsMotor.get();
  }

  /**
   * Sees how much current the wheels are taking (it will sky rocket if the motors are stalling)
   * Which means either a note is there or its hitting a wall
   * Either way, this will be used to move the intake back out in other commands
   * @return
   */
  // TODO BSM - Why theroetical? -> no clue lol, probably used to be a different formula that used voltage to calculate current
  public double getTheoreticalWheelCurrent() {
    double outputCurrent = intakeWheelsMotor.getOutputCurrent(); // * 12.6/RobotController.getBatteryVoltage();

    return outputCurrent;
  }

  //! as limit switch defaults to true
  public boolean getUpperLimitSwitch() {
    return upperLimitSwitch.get();
  }

  public boolean getLowerLimitSwitch() {
    return !lowerLimitSwitch.get();
  }

  /**
   * Resets hasNote
   * @param hasNote
   */
  public void resetNoteRegister(boolean hasNote) {
    this.hasNote = hasNote;
  }

  /**
   * Tells if the intake has detected a note in it
   * @return
   */
  public boolean hasNote() {
    return this.hasNote;
  }
}
