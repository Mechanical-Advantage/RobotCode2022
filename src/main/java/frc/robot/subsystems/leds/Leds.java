// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.leds.LedsIO.LedMode;

/**
 * Manages the pattern of the LEDs based on robot state. Note: This is NOT a WPILib subsystem,
 * meaning it cannot be required by commands. This prevents unexpected conflicts between commands,
 * potentially impacting normal robot operation.
 */
public class Leds {

  private final LedsIO io;

  // Robot state tracking
  private boolean autoAlert = false;
  private boolean shooting = false;
  private boolean driveTargeted = false;
  private boolean flywheelsReady = false;
  private boolean towerFull = false;
  private boolean intaking = false;
  private boolean climbing = false;

  public Leds(LedsIO io) {
    this.io = io;
  }

  /** Updates the current LED mode based on robot state. */
  public void update() {
    LedMode mode;
    if (DriverStation.isDisabled()) {
      mode = LedMode.DEFAULT_DISABLED;
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
    } else if (climbing) {
      mode = LedMode.CLIMBING;
    } else if (DriverStation.isAutonomous()) {
      mode = LedMode.DEFAULT_AUTO;
    } else {
      mode = LedMode.DEFAULT_TELEOP;
    }
    io.setMode(mode);
    Logger.getInstance().recordOutput("LEDMode", mode.toString());
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
