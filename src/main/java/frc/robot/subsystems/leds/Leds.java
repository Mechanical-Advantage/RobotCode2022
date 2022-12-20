// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.leds.LedsIO.LedMode;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Manages the pattern of the LEDs based on robot state. Note: This is NOT a WPILib subsystem,
 * meaning it cannot be required by commands. This prevents unexpected conflicts between commands,
 * potentially impacting normal robot operation.
 */
public class Leds {

  private final LedsIO io;
  private Supplier<String> demoModeSupplier = () -> "";

  // Robot state tracking
  private boolean climbing = false;
  private boolean climbFailure = false;
  private boolean climbSuccess = false;
  private boolean autoAlert = false;
  private boolean shooting = false;
  private boolean targeted = false;
  private boolean flywheelsReady = false;
  private int towerCount = 0;
  private boolean intaking = false;
  private boolean fallen = false;
  private boolean sameBattery = false;

  private Alliance alliance = Alliance.Invalid;

  public Leds(LedsIO io) {
    this.io = io;
  }

  public void setDemoModeSupplier(Supplier<String> demoModeSupplier) {
    this.demoModeSupplier = demoModeSupplier;
  }

  /** Updates the current LED mode based on robot state. */
  public void update() {
    // Update alliance color
    if (DriverStation.isFMSAttached()) {
      alliance = DriverStation.getAlliance();
    }

    // Get demo mode
    boolean demoTeam = false;
    boolean demoRainbow = false;
    switch (demoModeSupplier.get()) {
      case "Team Colors":
        demoTeam = true;
        break;
      case "Rainbow":
        demoRainbow = true;
        break;
    }

    // Select LED mode
    LedMode mode = LedMode.DISABLED_NEUTRAL;
    if (demoTeam || demoRainbow) { // Disable all other modes except shooting
      if (shooting) {
        mode = LedMode.SHOOTING;
      } else if (demoTeam) {
        mode = LedMode.DEMO_TEAM;
      } else if (demoRainbow) {
        mode = LedMode.DEMO_RAINBOW;
      }

    } else if (DriverStation.isDisabled()) {
      switch (alliance) {
        case Red:
          mode = LedMode.DISABLED_RED;
          break;
        case Blue:
          mode = LedMode.DISABLED_BLUE;
          break;
        default:
          mode = LedMode.DISABLED_NEUTRAL;
          break;
      }
    } else if (fallen) {
      mode = LedMode.FALLEN;
    } else if (climbing) {
      if (climbFailure) {
        mode = LedMode.CLIMB_FAILURE;
      } else if (climbSuccess) {
        mode = LedMode.CLIMB_SUCCESS;
      } else {
        mode = LedMode.CLIMB_NORMAL;
      }
    } else if (autoAlert) {
      mode = LedMode.AUTO_ALERT;
    } else if (shooting && DriverStation.isAutonomous()) {
      mode = LedMode.SHOOTING;
      // } else if (targeted && flywheelsReady && towerCount > 0
      // && DriverStation.isTeleop()) {
      // mode = LedMode.TARGETED;
    } else if (towerCount == 2 && DriverStation.isTeleop()) {
      mode = LedMode.TOWER_TWO_CARGO;
    } else if (towerCount == 1 && DriverStation.isTeleop()) {
      mode = LedMode.TOWER_ONE_CARGO;
    } else if (intaking && DriverStation.isTeleop()) {
      mode = LedMode.INTAKING;
    } else if (DriverStation.isAutonomous()) {
      mode = LedMode.DEFAULT_AUTO;
    } else {
      mode = LedMode.DEFAULT_TELEOP;
    }
    io.setMode(mode, sameBattery);
    Logger.getInstance().recordOutput("LEDMode", mode.toString());
  }

  public void setFallen(boolean active) {
    fallen = active;
  }

  public void setClimbing(boolean active) {
    if (active && !climbing) {
      climbFailure = false;
      climbSuccess = false;
    }
    climbing = active;
  }

  public void setClimbFailure(boolean active) {
    if (climbing) {
      climbFailure = active;
    }
  }

  public void setClimbSuccess(boolean active) {
    if (climbing) {
      climbSuccess = active;
    }
  }

  public void setAutoAlert(boolean active) {
    autoAlert = active;
  }

  public void setShooting(boolean active) {
    shooting = active;
  }

  public void setTargeted(boolean active) {
    targeted = active;
  }

  public void setFlywheelsReady(boolean active) {
    flywheelsReady = active;
  }

  public void setTowerCount(int count) {
    towerCount = count;
  }

  public void setIntaking(boolean active) {
    intaking = active;
  }

  public void setSameBatteryAlert(boolean active) {
    sameBattery = active;
  }
}
