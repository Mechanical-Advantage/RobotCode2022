// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

/** Controls LEDs directly from the RIO. */
public class LedsIORio implements LedsIO {
  private static final int length = 119;
  private static final double strobeDuration = 0.2; // How long is each flash
  private static final double rainbowFullLength = 40.0; // How many LEDs for a full cycle
  private static final double rainbowDuration = 0.25; // How long until the cycle repeats
  private static final double waveExponent = 0.4; // Controls the length of the transition
  private static final double waveSlowFullLength = 40.0; // How many LEDs for a full cycle
  private static final double waveSlowDuration = 3.0; // How long until the cycle repeats
  private static final double waveFastFullLength = 40.0; // How many LEDs for a full cycle
  private static final double waveFastDuration = 0.25; // How long until the cycle repeats

  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;

  public LedsIORio() {
    leds = new AddressableLED(0);
    buffer = new AddressableLEDBuffer(length);
    leds.setLength(length);
    leds.setData(buffer);
    leds.start();
  }

  @Override
  public void setMode(LedMode mode) {
    switch (mode) {
      case CLIMBING:
        rainbow();
        break;
      case AUTO_ALERT:
        solid(Color.kGreen);
        break;
      case SHOOTING:
        strobe(Color.kGold);
        break;
      case TARGETED:
        strobe(Color.kWhite);
        break;
      case TOWER_FULL:
        solid(Color.kGreen);
        break;
      case INTAKING:
        solid(Color.kBlue);
        break;
      case DEFAULT_AUTO:
        wave(Color.kGold, Color.kDarkBlue, waveFastFullLength,
            waveFastDuration);
        break;
      case DEFAULT_TELEOP:
        solid(Color.kBlack);
        break;
      case DEFAULT_DISABLED:
        wave(Color.kGold, Color.kDarkBlue, waveSlowFullLength,
            waveSlowDuration);
        break;
      default:
        solid(Color.kBlack);
        break;
    }
    leds.setData(buffer);
  }

  private void solid(Color color) {
    for (int i = 0; i < length; i++) {
      buffer.setLED(i, color);
    }
  }

  private void strobe(Color color) {
    boolean on = ((Timer.getFPGATimestamp() / strobeDuration) % 1.0) > 0.5;
    solid(on ? color : Color.kBlack);
  }

  private void rainbow() {
    double startHue =
        (1.0 - ((Timer.getFPGATimestamp() / rainbowDuration) % 1.0)) * 180.0;
    double hueDiffPerLed = 180.0 / rainbowFullLength;
    for (int i = 0; i < length; i++) {
      double hue = (startHue + (i * hueDiffPerLed)) % 180.0;
      buffer.setLED(i, Color.fromHSV((int) hue, 255, 255));
    }
  }

  private void wave(Color c1, Color c2, double fullLength, double duration) {
    double xStart = (1 - ((Timer.getFPGATimestamp() % duration) / duration))
        * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / fullLength;
    for (int i = 0; i < length; i++) {
      double x = xStart + (i * xDiffPerLed);
      double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
      if (Double.isNaN(ratio)) {
        ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
      }
      if (Double.isNaN(ratio)) {
        ratio = 0.5;
      }
      double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
      double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
      double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
      buffer.setLED(i, new Color(red, green, blue));
    }
  }
}