// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oi;

import edu.wpi.first.wpilibj.Joystick;

/** Class for the override switches on the OI console. */
public class OverrideOI {
  private Joystick overrides;

  /** Creates a dummy set of overrides if controller is not available. */
  public OverrideOI() {}

  /** Creates a set of overrides using the given controller port. */
  public OverrideOI(int port) {
    overrides = new Joystick(port);
  }

  public boolean getDriveDisable() {
    if (overrides == null) {
      return false;
    }
    return overrides.getRawButton(1);
  }

  public boolean getOpenLoop() {
    if (overrides == null) {
      return false;
    }
    return overrides.getRawButton(2);
  }

  public boolean getInternalEncoders() {
    if (overrides == null) {
      return false;
    }
    return overrides.getRawButton(3);
  }

  public VisionLEDMode getVisionLEDMode() {
    if (overrides != null) {
      if (overrides.getRawButton(4)) {
        return VisionLEDMode.ALWAYS_ON;
      }
      if (overrides.getRawButton(5)) {
        return VisionLEDMode.ALWAYS_OFF;
      }
    }
    return VisionLEDMode.AUTO;
  }

  public static enum VisionLEDMode {
    AUTO, ALWAYS_ON, ALWAYS_OFF
  }
}
