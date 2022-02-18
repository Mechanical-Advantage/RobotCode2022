// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hood.HoodIO.HoodIOInputs;

public class Hood extends SubsystemBase {
  private static final double moveTimeSecs = 1.0;

  private final HoodIO io;
  private final HoodIOInputs inputs = new HoodIOInputs();

  private final Timer movingTimer = new Timer();

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
  }

  public void setRaised(boolean raised) {
    if (raised != inputs.raised) {
      io.setRaised(raised);
      movingTimer.reset();
    }
  }

  public HoodState getState() {
    if (!movingTimer.hasElapsed(moveTimeSecs)) {
      return HoodState.MOVING;
    }
    return inputs.raised ? HoodState.RAISED : HoodState.LOWER;
  }

  public static enum HoodState {
    LOWER, RAISED, MOVING
  }
}
