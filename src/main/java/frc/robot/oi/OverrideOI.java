// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

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

  public VisionLedMode getVisionLedMode() {
    if (overrides != null) {
      if (overrides.getRawButton(4)) {
        return VisionLedMode.ALWAYS_ON;
      }
      if (overrides.getRawButton(5)) {
        return VisionLedMode.ALWAYS_OFF;
      }
    }
    return VisionLedMode.AUTO;
  }

  public boolean getClimbMode() {
    if (overrides == null) {
      return false;
    }
    return overrides.getRawButton(8);
  }

  public boolean getClimbOpenLoop() {
    if (overrides == null) {
      return false;
    }
    return overrides.getRawButton(9);
  }

  public boolean getCargoSensorDisable() {
    if (overrides == null) {
      return false;
    }
    return overrides.getRawButton(10);
  }

  public boolean getSimpleAutoAim() {
    if (overrides == null) {
      return false;
    }
    return overrides.getRawButton(11);
  }

  public boolean getShootPresets() {
    if (overrides == null) {
      return false;
    }
    return overrides.getRawButton(12);
  }

  public static enum VisionLedMode {
    AUTO,
    ALWAYS_ON,
    ALWAYS_OFF
  }
}
