// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.BlinkinLedDriver.BlinkinLedMode;

/** Manages the pattern of the LEDs based on robot state. */
public class LedSelector {
  private static final BlinkinLedMode defaultTeleopMode =
      BlinkinLedMode.BOTH_WAVES;
  private static final BlinkinLedMode defaultAutoMode =
      BlinkinLedMode.BOTH_BEATS;
  private static final BlinkinLedMode autoAlertMode = BlinkinLedMode.SOLID_LIME;
  private static final BlinkinLedMode shootingMode =
      BlinkinLedMode.FIXED_CHASE_BLUE;
  private static final BlinkinLedMode climbingMode =
      BlinkinLedMode.FIXED_CONFETTI;

  private final BlinkinLedDriver blinkin;

  // Robot state tracking
  private boolean autoAlert = false;
  private boolean shooting = false;
  private boolean climbing = false;

  public LedSelector(int blinkinChannel) {
    blinkin = new BlinkinLedDriver(blinkinChannel);
  }

  /** Updates the current LED mode based on robot state. */
  public void update() {
    if (DriverStation.isEnabled()) {
      BlinkinLedMode mode;
      if (autoAlert) {
        mode = autoAlertMode;
      } else if (shooting) {
        mode = shootingMode;
      } else if (climbing) {
        mode = climbingMode;
      } else if (DriverStation.isAutonomous()) {
        mode = defaultAutoMode;
      } else {
        mode = defaultTeleopMode;
      }
      blinkin.setMode(mode);
      Logger.getInstance().recordOutput("LEDMode", mode.toString());
    }
  }

  public void setAutoAlert(boolean active) {
    autoAlert = active;
  }

  public void setShooting(boolean active) {
    shooting = active;
  }

  public void setClimbing(boolean active) {
    climbing = active;
  }
}
