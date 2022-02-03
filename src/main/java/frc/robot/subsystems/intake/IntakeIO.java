// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Intake subsystem hardware interface. */
public interface IntakeIO {
  /** Contains all of the input data received from hardware. */
  public static class IntakeIOInputs implements LoggableInputs {
    public boolean extended = false;

    public double rollerPositionRad = 0.0;
    public double rollerVelocityRadPerSec = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double[] rollerCurrentAmps = new double[] {};
    public double[] rollerTempCelcius = new double[] {};

    public double hopperAppliedVolts = 0.0;
    public double[] hopperCurrentAmps = new double[] {};
    public double[] hopperTempCelcius = new double[] {};

    public void toLog(LogTable table) {
      table.put("Extended", extended);

      table.put("RollerPositionRad", rollerPositionRad);
      table.put("RollerVelocityRadPerSec", rollerVelocityRadPerSec);
      table.put("RollerAppliedVolts", rollerAppliedVolts);
      table.put("RollerCurrentAmps", rollerCurrentAmps);
      table.put("RollerTempCelcius", rollerTempCelcius);

      table.put("HopperAppliedVolts", hopperAppliedVolts);
      table.put("HopperCurrentAmps", hopperCurrentAmps);
      table.put("HopperTempCelcius", hopperTempCelcius);
    }

    public void fromLog(LogTable table) {
      extended = table.getBoolean("Extended", extended);

      rollerPositionRad =
          table.getDouble("RollerPositionRad", rollerPositionRad);
      rollerVelocityRadPerSec =
          table.getDouble("RollerVelocityRadPerSec", rollerVelocityRadPerSec);
      rollerAppliedVolts =
          table.getDouble("RollerAppliedVolts", rollerAppliedVolts);
      rollerCurrentAmps =
          table.getDoubleArray("RollerCurrentAmps", rollerCurrentAmps);
      rollerTempCelcius =
          table.getDoubleArray("RollerTempCelcius", rollerTempCelcius);

      hopperAppliedVolts =
          table.getDouble("HopperAppliedVolts", hopperAppliedVolts);
      hopperCurrentAmps =
          table.getDoubleArray("HopperCurrentAmps", hopperCurrentAmps);
      hopperTempCelcius =
          table.getDoubleArray("HopperTempCelcius", hopperTempCelcius);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run the roller open loop at the specified voltage. */
  public default void setRollerVoltage(double volts) {}

  /** Run the hopper open loop at the specified voltage. */
  public default void setHopperVoltage(double volts) {}

  /** Enable or disable brake mode on the roller. */
  public default void setRollerBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the hopper. */
  public default void setHopperBrakeMode(boolean enable) {}

  /** Set solenoid state. */
  public default void setExtended(boolean extended) {}
}
