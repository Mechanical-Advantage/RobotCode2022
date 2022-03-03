// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.leds.LedsIO.LedMode;

/**
 * Manages the pattern of the LEDs based on robot state. Note: This is NOT a WPILib subsystem,
 * meaning it cannot be required by commands. This prevents unexpected conflicts between commands,
 * potentially impacting normal robot operation.
 */
public class Leds {

  private final LedsIO io;

  // Robot state tracking
  private boolean climbing = false;
  private boolean autoAlert = false;
  private boolean shooting = false;
  private boolean driveTargeted = false;
  private boolean flywheelsReady = false;
  private boolean towerFull = false;
  private boolean intaking = false;

  private Alliance alliance = Alliance.Invalid;

  public Leds(LedsIO io) {
    this.io = io;
  }

  /** Updates the current LED mode based on robot state. */
  public void update() {
    // Update alliance color
    if (DriverStation.isFMSAttached()) {
      alliance = DriverStation.getAlliance();
    }

    // Select LED mode
    LedMode mode;
    if (DriverStation.isDisabled()) {
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
    } else if (climbing) {
      mode = LedMode.CLIMBING;
    } else if (autoAlert) {
      mode = LedMode.AUTO_ALERT;
    } else if (shooting) {
      mode = LedMode.SHOOTING;
    } else if (driveTargeted && flywheelsReady && DriverStation.isTeleop()) {
      mode = LedMode.TARGETED;
    } else if (towerFull && DriverStation.isTeleop()) {
      mode = LedMode.TOWER_FULL;
    } else if (intaking && DriverStation.isTeleop()) {
      mode = LedMode.INTAKING;
    } else if (DriverStation.isAutonomous()) {
      mode = LedMode.DEFAULT_AUTO;
    } else {
      mode = LedMode.DEFAULT_TELEOP;
    }
    io.setMode(mode);
    Logger.getInstance().recordOutput("LEDMode", mode.toString());
  }

  public void setClimbing(boolean active) {
    climbing = active;
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
}
