// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oi;

/** Class for the override switches on the OI console. */
public class OverrideOI {

  /** Creates a dummy set of overrides if controller is not available. */
  public OverrideOI() {}

  /** Creates a set of overrides using the given controller port. */
  public OverrideOI(int port) {}

  public boolean getDriveDisable() {
    return false;
  }

  public boolean getOpenLoop() {
    return false;
  }

  public VisionLEDMode getVisionLEDMode() {
    return VisionLEDMode.AUTO;
  }

  public static enum VisionLEDMode {
    AUTO, ALWAYS_ON, ALWAYS_OFF
  }
}
