// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Hood subsystem hardware interface. */
public interface HoodIO {
  /** Contains all of the input data received from hardware. */
  public static class HoodIOInputs implements LoggableInputs {
    public boolean raised = false;

    public void toLog(LogTable table) {
      table.put("Raised", raised);
    }

    public void fromLog(LogTable table) {
      raised = table.getBoolean("Raised", raised);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(HoodIOInputs inputs) {}

  /** Sets the hood position. */
  public default void setRaised(boolean raised) {}
}
