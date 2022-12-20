// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.flywheels;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Flywheels subsystem hardware interface. */
public interface FlywheelsIO {
  /** Contains all of the input data received from hardware. */
  public static class FlywheelsIOInputs implements LoggableInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};

    public void toLog(LogTable table) {
      table.put("PositionRad", positionRad);
      table.put("VelocityRadPerSec", velocityRadPerSec);
      table.put("AppliedVolts", appliedVolts);
      table.put("CurrentAmps", currentAmps);
      table.put("TempCelcius", tempCelcius);
    }

    public void fromLog(LogTable table) {
      positionRad = table.getDouble("PositionRad", positionRad);
      velocityRadPerSec = table.getDouble("VelocityRadPerSec", velocityRadPerSec);
      appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
      currentAmps = table.getDoubleArray("CurrentAmps", currentAmps);
      tempCelcius = table.getDoubleArray("TempCelcius", tempCelcius);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FlywheelsIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(double velocityRadPerSec, double ffVolts) {}

  /** Enable or disable brake mode. */
  public default void setBrakeMode(boolean enable) {}

  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}
}
