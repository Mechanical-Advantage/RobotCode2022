// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.leds;

import frc.robot.util.BlinkinLedDriver;
import frc.robot.util.BlinkinLedDriver.BlinkinLedMode;
import java.util.HashMap;
import java.util.Map;

/** Maps LED mode to Blinkin patterns. */
public class LedsIOBlinkin implements LedsIO {
  private static final Map<LedMode, BlinkinLedMode> modeLookup = new HashMap<>();

  static {
    modeLookup.put(LedMode.FALLEN, BlinkinLedMode.FIXED_STROBE_WHITE);
    modeLookup.put(LedMode.CLIMB_NORMAL, BlinkinLedMode.FIXED_RAINBOW_PARTY);
    modeLookup.put(LedMode.CLIMB_FAILURE, BlinkinLedMode.FIXED_BREATH_BLUE);
    modeLookup.put(LedMode.CLIMB_SUCCESS, BlinkinLedMode.FIXED_BREATH_RED);
    modeLookup.put(LedMode.AUTO_ALERT, BlinkinLedMode.SOLID_GREEN);
    modeLookup.put(LedMode.SHOOTING, BlinkinLedMode.ONE_STROBE);
    modeLookup.put(LedMode.TARGETED, BlinkinLedMode.FIXED_STROBE_WHITE);
    modeLookup.put(LedMode.TOWER_TWO_CARGO, BlinkinLedMode.SOLID_GREEN);
    modeLookup.put(LedMode.TOWER_ONE_CARGO, BlinkinLedMode.SOLID_VIOLET);
    modeLookup.put(LedMode.INTAKING, BlinkinLedMode.SOLID_BLUE);
    modeLookup.put(LedMode.DEFAULT_AUTO, BlinkinLedMode.TWO_HEARTBEAT_FAST);
    modeLookup.put(LedMode.DEFAULT_TELEOP, BlinkinLedMode.SOLID_BLACK);
  }

  private final BlinkinLedDriver blinkin;

  public LedsIOBlinkin() {
    blinkin = new BlinkinLedDriver(0);
  }

  @Override
  public void setMode(LedMode mode, boolean sameBattery) {
    if (modeLookup.containsKey(mode)) {
      blinkin.setMode(modeLookup.get(mode));
    } else {
      blinkin.setMode(BlinkinLedMode.SOLID_BLACK);
    }
  }
}
