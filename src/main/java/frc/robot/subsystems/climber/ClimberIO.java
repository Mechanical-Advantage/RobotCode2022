// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** climber subsystem hardware interface. */
public interface ClimberIO {
  /** Contains all of the input data received from hardware. */
  public static class ClimberIOInputs implements LoggableInputs {
    public boolean unlocked = false;
    public boolean limitSwitchLeft = false;
    public boolean limitSwitchRight = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};

    public void toLog(LogTable table) {
      table.put("Unlocked", unlocked);
      table.put("LimitLeft", limitSwitchLeft);
      table.put("LimitRight", limitSwitchRight);
      table.put("PositionRad", positionRad);
      table.put("VelocityRadPerSec", velocityRadPerSec);
      table.put("AppliedVolts", appliedVolts);
      table.put("CurrentAmps", currentAmps);
      table.put("TempCelcius", tempCelcius);
    }

    public void fromLog(LogTable table) {
      unlocked = table.getBoolean("Unlocked", unlocked);
      limitSwitchLeft = table.getBoolean("LimitActiveLeft", limitSwitchLeft);
      limitSwitchRight = table.getBoolean("LimitActiveRight", limitSwitchRight);
      positionRad = table.getDouble("PositionRad", positionRad);
      velocityRadPerSec = table.getDouble("VelocityRadPerSec", velocityRadPerSec);
      appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
      currentAmps = table.getDoubleArray("CurrentAmps", currentAmps);
      tempCelcius = table.getDoubleArray("TempCelcius", tempCelcius);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Enable or disable brake mode. */
  public default void setBrakeMode(boolean enable) {}

  /** Lock or unlock pistons. */
  public default void setUnlocked(boolean unlocked) {}
}
