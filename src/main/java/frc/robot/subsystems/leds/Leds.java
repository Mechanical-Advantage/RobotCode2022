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
  private boolean climbFailure = false;
  private boolean climbSuccess = false;
  private boolean autoAlert = false;
  private boolean shooting = false;
  private boolean targeted = false;
  private boolean flywheelsReady = false;
  private int towerCount = 0;
  private boolean intaking = false;
  private boolean fallen = false;

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
    // if (DriverStation.isDisabled()) {
    // switch (alliance) {
    // case Red:
    // mode = LedMode.DISABLED_RED;
    // break;
    // case Blue:
    // mode = LedMode.DISABLED_BLUE;
    // break;
    // default:
    // mode = LedMode.DISABLED_NEUTRAL;
    // break;
    // }
    // } else if (fallen) {
    // mode = LedMode.FALLEN;
    // } else if (climbing) {
    // if (climbFailure) {
    // mode = LedMode.CLIMB_FAILURE;
    // } else if (climbSuccess) {
    // mode = LedMode.CLIMB_SUCCESS;
    // } else {
    // mode = LedMode.CLIMB_NORMAL;
    // }
    // } else if (autoAlert) {
    // mode = LedMode.AUTO_ALERT;
    // } else if (shooting && DriverStation.isAutonomous()) {
    // mode = LedMode.SHOOTING;
    // // } else if (targeted && flywheelsReady && towerCount > 0
    // // && DriverStation.isTeleop()) {
    // // mode = LedMode.TARGETED;
    // } else if (towerCount == 2 && DriverStation.isTeleop()) {
    // mode = LedMode.TOWER_TWO_CARGO;
    // } else if (towerCount == 1 && DriverStation.isTeleop()) {
    // mode = LedMode.TOWER_ONE_CARGO;
    // } else if (intaking && DriverStation.isTeleop()) {
    // mode = LedMode.INTAKING;
    // } else if (DriverStation.isAutonomous()) {
    // mode = LedMode.DEFAULT_AUTO;
    // } else {
    // mode = LedMode.DEFAULT_TELEOP;
    // }
    if (shooting) {
      mode = LedMode.SHOOTING;
    } else {
      mode = LedMode.CLIMB_NORMAL;
    }
    io.setMode(mode);
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
}
