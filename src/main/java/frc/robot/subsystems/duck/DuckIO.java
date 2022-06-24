// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.duck;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.subsystems.duck.Duck.DuckSound;

/** Duck subsystem hardware interface. */
public interface DuckIO {
  /** Contains all of the input data received from hardware. */
  public static class DuckIOInputs implements LoggableInputs {
    public double appliedVolts = 0.0;

    public void toLog(LogTable table) {
      table.put("AppliedVolts", appliedVolts);
    }

    public void fromLog(LogTable table) {
      appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(DuckIOInputs inputs) {}

  /** Run the duck open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Activates the pin for the specified sound (or none if null). */
  public default void setActive(DuckSound sound) {}
}