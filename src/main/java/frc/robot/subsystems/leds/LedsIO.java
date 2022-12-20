// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.leds;

/** Leds hardware interface. */
public interface LedsIO {

  /** Sets the current LED mode. */
  public default void setMode(LedMode mode, boolean sameBattery) {}

  /**
   * Possible LED modes based on robot state, IO implementations should select an appropriate
   * pattern.
   */
  public static enum LedMode {
    FALLEN,
    CLIMB_NORMAL,
    CLIMB_FAILURE,
    CLIMB_SUCCESS,
    AUTO_ALERT,
    SHOOTING,
    TARGETED,
    TOWER_TWO_CARGO,
    TOWER_ONE_CARGO,
    INTAKING,
    DEFAULT_AUTO,
    DEFAULT_TELEOP,
    DISABLED_RED,
    DISABLED_BLUE,
    DISABLED_NEUTRAL,
    DEMO_TEAM,
    DEMO_RAINBOW
  }
}
