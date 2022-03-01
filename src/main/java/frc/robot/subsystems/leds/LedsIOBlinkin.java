// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import java.util.HashMap;
import java.util.Map;

import frc.robot.util.BlinkinLedDriver;
import frc.robot.util.BlinkinLedDriver.BlinkinLedMode;

/** Maps LED mode to Blinkin patterns. */
public class LedsIOBlinkin implements LedsIO {
  private static final Map<LedMode, BlinkinLedMode> modeLookup =
      new HashMap<>();

  static {
    modeLookup.put(LedMode.AUTO_ALERT, BlinkinLedMode.SOLID_GREEN);
    modeLookup.put(LedMode.SHOOTING, BlinkinLedMode.ONE_STROBE);
    modeLookup.put(LedMode.TARGETED, BlinkinLedMode.FIXED_STROBE_WHITE);
    modeLookup.put(LedMode.TOWER_FULL, BlinkinLedMode.SOLID_GREEN);
    modeLookup.put(LedMode.INTAKING, BlinkinLedMode.SOLID_BLUE);
    modeLookup.put(LedMode.CLIMBING, BlinkinLedMode.FIXED_RAINBOW_PARTY);
    modeLookup.put(LedMode.DEFAULT_AUTO, BlinkinLedMode.TWO_HEARTBEAT_FAST);
    modeLookup.put(LedMode.DEFAULT_TELEOP, BlinkinLedMode.SOLID_BLACK);
  }

  private final BlinkinLedDriver blinkin;

  public LedsIOBlinkin() {
    blinkin = new BlinkinLedDriver(0);
  }

  @Override
  public void setMode(LedMode mode) {
    if (modeLookup.containsKey(mode)) {
      blinkin.setMode(modeLookup.get(mode));
    } else {
      blinkin.setMode(BlinkinLedMode.SOLID_BLACK);
    }
  }
}
