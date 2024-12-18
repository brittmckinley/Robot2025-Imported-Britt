// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DebugConstants;

public class Climber extends SubsystemBase {

  private PneumaticHub m_pHub;
  private DoubleSolenoid climberSolenoid;

  // Don't really need to add the compressor - it is there automatically.
  // private Compressor climbCompress or;

  /** Creates a new Climber. */
  public Climber() {
    m_pHub = new PneumaticHub(Constants.ClimberConstants.hubPort);

    climberSolenoid =
      m_pHub.makeDoubleSolenoid(
        ClimberConstants.forwardChannel,
        ClimberConstants.reverseChannel
      );
    m_pHub.enableCompressorDigital();
    climberSolenoid.set(Value.kForward);
  }

  @Override
  public void periodic() {
    if (DebugConstants.enableSmartDash) {
      // Add buttons to set the double
      // SmartDashboard.setDefaultBoolean("Set Off", false);
      // SmartDashboard.setDefaultBoolean("Set Forward", false);
      // SmartDashboard.setDefaultBoolean("Set Reverse", false);
    }
  }

  public void toggleClimber() {
    climberSolenoid.toggle();
  }

  public void closeClimber() {
    climberSolenoid.set(DoubleSolenoid.Value.kReverse);
    SmartDashboard.putString("Climber", "Close");
  }

  public void openClimber() {
    climberSolenoid.set(DoubleSolenoid.Value.kForward);
    SmartDashboard.putString("Climber", "Open");
  }

  public void stopClimber() {
    climberSolenoid.set(DoubleSolenoid.Value.kOff);
  }
  // Not really needed - but left in just in case
  /*
  // Compressor Functions
  public void enableCompressor()
  {
    m_pHub.enableCompressorDigital();
  }

  public void disableCompressor()
  {
    m_pHub.disableCompressor();
  }

  public boolean isEnabled()
  {
    return m_pHub.getCompressor();
  }

  public boolean getFull() 
  {
    return climbCompressor.getPressureSwitchValue();
  }

  public double getCurrent()
  {
    return climbCompressor.getCurrent();
  }

  public double getPressure()
  {
    return climbCompressor.getPressure();
  }
  */
}
