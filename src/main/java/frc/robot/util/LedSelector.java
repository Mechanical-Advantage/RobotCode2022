// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.BlinkinLedDriver.BlinkinLedMode;

/** Manages the pattern of the LEDs based on robot state. */
public class LedSelector {
  public static final boolean testMode = false; // Select LED mode from the dashboard

  private static final BlinkinLedMode defaultTeleopMode =
      BlinkinLedMode.SOLID_BLACK;
  private static final BlinkinLedMode defaultAutoMode =
      BlinkinLedMode.TWO_HEARTBEAT_FAST;
  private static final BlinkinLedMode autoAlertMode =
      BlinkinLedMode.SOLID_GREEN;
  private static final BlinkinLedMode shootingMode = BlinkinLedMode.ONE_STROBE;
  private static final BlinkinLedMode targetedMode =
      BlinkinLedMode.FIXED_STROBE_WHITE;
  private static final BlinkinLedMode towerFullMode =
      BlinkinLedMode.SOLID_GREEN;
  private static final BlinkinLedMode intakingMode = BlinkinLedMode.SOLID_BLUE;
  private static final BlinkinLedMode climbingMode =
      BlinkinLedMode.FIXED_RAINBOW_PARTY;

  private final BlinkinLedDriver blinkin;
  private Supplier<String> testModeSupplier;

  // Robot state tracking
  private boolean autoAlert = false;
  private boolean shooting = false;
  private boolean driveTargeted = false;
  private boolean flywheelsReady = false;
  private boolean towerFull = false;
  private boolean intaking = false;
  private boolean climbing = false;

  public LedSelector(int blinkinChannel) {
    blinkin = new BlinkinLedDriver(blinkinChannel);

    if (testMode) {
      new Alert("LED test mode active, pattern must be set from the dashboard.",
          AlertType.INFO).set(true);
    }
  }

  public void setTestModeSupplier(Supplier<String> testModeSupplier) {
    this.testModeSupplier = testModeSupplier;
  }

  /** Updates the current LED mode based on robot state. */
  public void update() {
    if (DriverStation.isEnabled()) {
      BlinkinLedMode mode;
      if (testMode) {
        try {
          mode = BlinkinLedMode.valueOf(testModeSupplier.get());
        } catch (IllegalArgumentException e) {
          mode = BlinkinLedMode.SOLID_BLACK;
        }
      } else if (autoAlert) {
        mode = autoAlertMode;
      } else if (shooting) {
        mode = shootingMode;
      } else if (driveTargeted && flywheelsReady && DriverStation.isTeleop()) {
        mode = targetedMode;
      } else if (towerFull && DriverStation.isTeleop()) {
        mode = towerFullMode;
      } else if (intaking && DriverStation.isTeleop()) {
        mode = intakingMode;
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

  public void setDriveTargeted(boolean active) {
    driveTargeted = active;
  }

  public void setFlywheelsReady(boolean active) {
    flywheelsReady = active;
  }

  public void setTowerFull(boolean active) {
    towerFull = active;
  }

  public void setIntaking(boolean active) {
    intaking = active;
  }

  public void setClimbing(boolean active) {
    climbing = active;
  }
}
