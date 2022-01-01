// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oi;

/**
 * Utility class for selecting the appropriate OI implementations based on the connected joysticks.
 */
public class OISelector {

  private OISelector() {}

  /**
   * Returns whether the connected joysticks have changed since the last time this method was
   * called.
   */
  public static boolean didJoysticksChange() {
    return false;
  }

  /**
   * Instantiates and returns an appropriate override OI object based on the connected joysticks.
   */
  public static OverrideOI findOverrideOI() {
    return new OverrideOI();
  }

  /**
   * Instantiates and returns an appropriate handheld OI object (driver and operator) based on the
   * connected joysticks.
   */
  public static HandheldOI findHandheldOI() {
    return new HandheldOI() {};
  }
}
