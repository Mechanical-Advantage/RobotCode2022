// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

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
