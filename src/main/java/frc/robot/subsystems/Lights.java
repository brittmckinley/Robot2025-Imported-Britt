// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import java.util.Random;

public class Lights extends SubsystemBase {

  private static int m_ledLen = 290;
  private static int m_ledUnderStart = 0;
  private static int m_ledUnderEnd = 151;
  private static int m_ledUprLeftStart = 152;
  private static int m_ledUprLeftEnd = 184;
  private static int m_ledShtRightStart = 185;
  private static int m_ledShtRightEnd = 205;
  private static int m_ledShtUnderStart = 206;
  private static int m_ledShtUnderEnd = 235;
  private static int m_ledShtLeftStart = 236;
  private static int m_ledShtLeftEnd = 256;
  private static int m_ledUprRightStart = 257;
  private static int m_ledUprRightEnd = 289;

  private static AddressableLED m_leds = new AddressableLED(0);
  private static AddressableLEDBuffer m_ledBuff = new AddressableLEDBuffer(
    m_ledLen
  );
  private static Optional<DriverStation.Alliance> m_color;
  private boolean runningIdleMode = true;
  private static int stripOffset = 0;

  private static Timer lightTimer;

  /** Creates a new Leds. */
  public Lights() {
    m_leds.setLength(m_ledLen);
    m_leds.start();
    lightTimer = new Timer();
    lightTimer.start();
  }

  public boolean getIdleMode() {
    return this.runningIdleMode;
  }

  public void setIdleMode(boolean mode) {
    this.runningIdleMode = mode;
  }

  public static void rainbow() {
    int m_rainbowFirstPixelHue = 0;
    int offset = 0; // Assuming offset is a constant value
    int length = m_ledBuff.getLength(); // Calculate length once to avoid repeated method calls

    // Calculate the hue offset
    int hueOffset = (offset * 180) / length;

    for (var i = 0; i < length; i++) {
      final var hue =
        (m_rainbowFirstPixelHue + i * 180 / length + hueOffset) % 180;
      m_ledBuff.setHSV(i, hue, 255, 128);
    }

    m_rainbowFirstPixelHue += 8;
    m_rainbowFirstPixelHue %= 180;
    m_leds.setData(m_ledBuff);
  }

  public static void allianceLeds() {
    m_color = DriverStation.getAlliance();

    if (m_color.isEmpty()) {
      return;
    }

    for (int ledIdx = 0; ledIdx < m_ledLen; ledIdx++) {
      if (m_color.get() == DriverStation.Alliance.Blue) {
        m_ledBuff.setRGB(ledIdx, 0, 0, 0xFF);
      } else {
        m_ledBuff.setRGB(ledIdx, 0x7F, 0, 0);
      }
    }

    m_leds.setData(m_ledBuff);
  }

  private static int groups = 18;

  public static void fancyAllianceLeds() {
    m_color = DriverStation.getAlliance();

    if (m_color.isEmpty()) {
      return;
    }

    int groupLen = m_ledLen / groups;

    for (int g = 0; g < groups; g++) {
      for (int ledIdx = groupLen * g; ledIdx < (g + 1) * groupLen; ledIdx++) {
        if (m_color.get() == DriverStation.Alliance.Blue) {
          if (g % 3 == 0) {
            m_ledBuff.setRGB((ledIdx + stripOffset) % m_ledLen, 0, 0, 0xFF);
          } else if (g % 3 == 1) {
            m_ledBuff.setRGB((ledIdx + stripOffset) % m_ledLen, 0, 0xFF, 0xFF);
          } else {
            m_ledBuff.setRGB((ledIdx + stripOffset) % m_ledLen, 0, 0x7F, 0xFF);
          }
        } else {
          if (g % 3 == 0) {
            m_ledBuff.setRGB((ledIdx + stripOffset) % m_ledLen, 0xFF, 0, 0x01);
          } else if (g % 3 == 1) {
            m_ledBuff.setRGB((ledIdx + stripOffset) % m_ledLen, 0xFF, 0, 0x04);
          } else {
            m_ledBuff.setRGB((ledIdx + stripOffset) % m_ledLen, 0xFF, 0, 0x08);
          }
        }
      }
    }

    m_leds.setData(m_ledBuff);
  }

  public static void fancySplitRainbow() {
    int m_rainbowFirstPixelHue = 0;

    // For every pixel
    for (var i = 0; i < m_ledBuff.getLength() / 2; i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue =
        (
          m_rainbowFirstPixelHue +
          ((i + stripOffset) * 180 / m_ledBuff.getLength())
        ) %
        180;

      // Set the value
      m_ledBuff.setHSV(i, hue, 255, 128);
      m_ledBuff.setHSV(m_ledBuff.getLength() - 1 - i, hue, 255, 128);
    }

    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;

    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }

  static int rippleTicker = 0;

  static double[] rippleEffectArraySeed = new double[291];
  static double[] rippleEffectArrayResult = new double[291];
  static Random seedArray = new Random();

  public static void setAllianceRippleEffect(int start, int end) {
    int length = end - start + 1;

    // Seed ones randomly in the array
    for (int arrayIdx = start; arrayIdx <= end; arrayIdx++) {
      if (seedArray.nextDouble() < 200 * 0.2 / m_ledLen / 20) { // Adjust the probability to seed ones
        rippleEffectArraySeed[arrayIdx] = 1;
      }
      if (seedArray.nextDouble() < 280 * 0.2 / m_ledLen / 20) { // Adjust the probability to seed zeros
        rippleEffectArraySeed[arrayIdx] = 0;
      }
    }

    if (rippleTicker % 3 == 1) {
      // Apply convolution blur to create the ripple effect
      for (int arrayIdx = start; arrayIdx <= end; arrayIdx++) {
        // Apply blur by averaging neighboring values
        double sum = rippleEffectArraySeed[arrayIdx];
        int count = 1;

        if (arrayIdx > 0) {
          sum += rippleEffectArraySeed[arrayIdx - 1];
          count++;
        }
        if (arrayIdx < 289) {
          sum += rippleEffectArraySeed[arrayIdx + 1];
          count++;
        }
        if (arrayIdx == 0) {
          sum += rippleEffectArraySeed[289];
          count++;
        }
        if (arrayIdx == 289) {
          sum += rippleEffectArraySeed[0];
          count++;
        }

        rippleEffectArrayResult[arrayIdx] = sum / count; // Update the value based on the average of neighbors

        if (rippleEffectArrayResult[arrayIdx] < .10) {
          rippleEffectArrayResult[arrayIdx] = 0;
        }
      }
    }

    // Apply the ripple effect to the LED strip
    for (int ledIdx = start; ledIdx <= end; ledIdx++) {
      rippleEffectArraySeed[ledIdx] = rippleEffectArrayResult[ledIdx];
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        m_ledBuff.setHSV(
          ledIdx,
          120 - (int) (rippleEffectArrayResult[ledIdx] * 70),
          255 - (int) (rippleEffectArrayResult[ledIdx] * 20),
          50 + (int) (rippleEffectArrayResult[ledIdx] * 50)
        );
      } else {
        m_ledBuff.setHSV(
          ledIdx,
          0 + (int) (rippleEffectArrayResult[ledIdx] * 15),
          255 - (int) (rippleEffectArrayResult[ledIdx] * 7),
          70 + (int) (rippleEffectArrayResult[ledIdx] * 30)
        );
      }
    }
  }

  // Setting specific led effects

  // Alliance LEDs in set intervals
  public void setAllianceRippleEffectUnder() {
    setAllianceRippleEffect(m_ledUnderStart, m_ledUnderEnd);
  }

  public void setAllianceRippleEffectUpr() {
    setAllianceRippleEffect(m_ledUprLeftStart, m_ledUprLeftEnd);
    setAllianceRippleEffect(m_ledUprRightStart, m_ledUprRightEnd);
  }

  public void setAllianceRippleEffectSht() {
    setAllianceRippleEffect(m_ledShtLeftStart, m_ledShtLeftEnd);
    setAllianceRippleEffect(m_ledShtRightStart, m_ledShtRightEnd);
    setAllianceRippleEffect(m_ledShtUnderStart, m_ledShtUnderEnd);
  }

  // Status LEDs
  public void setLedsToColor(int start, int end, int r, int g, int b) {
    for (int ledIdx = start; ledIdx <= end; ledIdx++) {
      m_ledBuff.setRGB(ledIdx, r, g, b);
    }
  }

  public void setStatusLedsUpr(int r, int g, int b) {
    setLedsToColor(m_ledUprLeftStart, m_ledUprLeftEnd, r, g, b);
    setLedsToColor(m_ledUprRightStart, m_ledUprRightEnd, r, g, b);
  }

  public void setStatusLedsSht(int r, int g, int b) {
    setLedsToColor(m_ledShtLeftStart, m_ledShtLeftEnd, r, g, b);
    setLedsToColor(m_ledShtRightStart, m_ledShtRightEnd, r, g, b);
    setLedsToColor(m_ledShtUnderStart, m_ledShtUnderEnd, r, g, b);
  }

  // Status LEDs Applied
  public void setStatusLedsIntakeDeployed() {
    int blinkInterval = 200; // Blink interval in milliseconds

    int currentTime = (int) (lightTimer.get() * 1000); // (in Ms)
    boolean isOn = currentTime % (blinkInterval * 2) < blinkInterval;
    for (int ledIdx = m_ledUprLeftStart; ledIdx < m_ledUprLeftEnd; ledIdx++) {
      if (isOn) {
        setStatusLedsUpr(0xFF, 0x38, 0x00);
      } else {
        setStatusLedsUpr(0, 0, 0);
      }

      isOn = !isOn;
    }
  }

  private int statusLedsOffset = 0;

  public void setStatusLedsIntakeHasNote() {
    if (statusLedsOffset < m_ledUprRightEnd - m_ledUprRightStart + 1) {
      m_ledBuff.setRGB(m_ledUprLeftStart + statusLedsOffset, 0, 200, 30);

      m_ledBuff.setRGB(m_ledUprRightEnd - statusLedsOffset, 0, 200, 30);

      if (currentTick % 1 == 0) {
        statusLedsOffset++;
      }
    }
  }

  public void resetStatusLedsOffset() {
    statusLedsOffset = 0;
  }

  public void setStatusLedsShtHasNote() {
    setStatusLedsSht(0xFF, 0x38, 0x00);
  }

  // Misc
  int looppertick = 2;
  int currentTick = 0;

  static int upAndDownTicker = 0;
  int upAndDownMax = 60;
  boolean goUp = true;

  @Override
  public void periodic() {
    //offset runs
    stripOffset++;
    stripOffset %= m_ledLen;

    if (goUp) {
      upAndDownTicker++;
      if (upAndDownTicker >= upAndDownMax) goUp = false;
    } else {
      upAndDownTicker--;
      if (upAndDownTicker <= -upAndDownMax) goUp = true;
    }

    //runs if command isnt running
    if (runningIdleMode) {
      if (DriverStation.isTeleop()) {
        setAllianceRippleEffect(0, m_ledLen - 1);
      } else {
        fancySplitRainbow();
      }
    }
    m_leds.setData(m_ledBuff);
    currentTick++;
    rippleTicker++;
  }
}
