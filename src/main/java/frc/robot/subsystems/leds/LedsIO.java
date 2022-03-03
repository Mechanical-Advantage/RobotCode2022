// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

/** Leds hardware interface. */
public interface LedsIO {

  /** Sets the current LED mode. */
  public default void setMode(LedMode mode) {}

  /**
   * Possible LED modes based on robot state, IO implementations should select an appropriate
   * pattern.
   */
  public static enum LedMode {
    CLIMBING, AUTO_ALERT, SHOOTING, TARGETED, TOWER_FULL, INTAKING, DEFAULT_AUTO, DEFAULT_TELEOP, DISABLED_RED, DISABLED_BLUE, DISABLED_NEUTRAL
  }
}
