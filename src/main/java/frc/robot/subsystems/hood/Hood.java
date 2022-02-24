// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hood.HoodIO.HoodIOInputs;

public class Hood extends SubsystemBase {
  private static final double moveTimeSecs = 0.5;

  private final HoodIO io;
  private final HoodIOInputs inputs = new HoodIOInputs();

  private final Timer movingTimer = new Timer();
  private boolean resetComplete = false; // Reset on first enable
  private boolean raised = false;
  private boolean shootPosition = false;

  /** Creates a new Kicker. */
  public Hood(HoodIO io) {
    this.io = io;
    movingTimer.reset();
    movingTimer.start();
    io.setRaised(false);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Hood", inputs);

    // Start moving timer on first reset
    if (DriverStation.isEnabled() && !resetComplete) {
      movingTimer.reset();
      resetComplete = true;
    }

    // Log current state
    Logger.getInstance().recordOutput("HoodState", getState().toString());
  }

  /** Sets the hood position. */
  public void moveToPosition(boolean raised) {
    if (raised != this.raised && DriverStation.isEnabled()) {
      io.setRaised(raised);
      movingTimer.reset();
      this.raised = raised;
    }
  }

  /** Saves a requested position for shooting, but doesn't cause any movement. */
  public void requestShootPosition(boolean raised) {
    shootPosition = raised;
  }

  /** Moves to the current requested shoot position. */
  public void moveToShootPosition() {
    moveToPosition(shootPosition);
  }

  public HoodState getState() {
    if (!movingTimer.hasElapsed(moveTimeSecs)) {
      return HoodState.MOVING;
    }
    if (!resetComplete) {
      return HoodState.UNKNOWN;
    }
    return raised ? HoodState.RAISED : HoodState.LOWER;
  }

  public static enum HoodState {
    LOWER, RAISED, MOVING, UNKNOWN
  }
}
