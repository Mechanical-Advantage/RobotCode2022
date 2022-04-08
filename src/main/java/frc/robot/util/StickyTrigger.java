// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** A trigger that can be explicitly set to active or inactive and retains its state. */
public class StickyTrigger extends Trigger {
  private boolean active = false;

  @Override
  public boolean get() {
    return active;
  }

  public void setActive() {
    active = true;
  }

  public void setInactive() {
    active = false;
  }
}
