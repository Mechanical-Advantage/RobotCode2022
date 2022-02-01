// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheels;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Flywheels subsystem hardware interface. */
public interface FlywheelsIO {
  /** Contains all of the input data received from hardware. */
  public static class FlywheelsIOInputs implements LoggableInputs {
    public double bigPositionRad = 0.0;
    public double bigVelocityRadPerSec = 0.0;
    public double bigAppliedVolts = 0.0;
    public double[] bigCurrentAmps = new double[] {};
    public double[] bigTempCelcius = new double[] {};

    public double littlePositionRad = 0.0;
    public double littleVelocityRadPerSec = 0.0;
    public double littleAppliedVolts = 0.0;
    public double[] littleCurrentAmps = new double[] {};
    public double[] littleTempCelcius = new double[] {};

    public void toLog(LogTable table) {
      table.put("BigPositionRad", bigPositionRad);
      table.put("BigVelocityRadPerSec", bigVelocityRadPerSec);
      table.put("BigAppliedVolts", bigAppliedVolts);
      table.put("BigCurrentAmps", bigCurrentAmps);
      table.put("BigTempCelcius", bigTempCelcius);

      table.put("LittlePositionRad", littlePositionRad);
      table.put("LittleVelocityRadPerSec", littleVelocityRadPerSec);
      table.put("LittleAppliedVolts", littleAppliedVolts);
      table.put("LittleCurrentAmps", littleCurrentAmps);
      table.put("LittleTempCelcius", littleTempCelcius);
    }

    public void fromLog(LogTable table) {
      bigPositionRad = table.getDouble("BigPositionRad", bigPositionRad);
      bigVelocityRadPerSec =
          table.getDouble("BigVelocityRadPerSec", bigVelocityRadPerSec);
      bigAppliedVolts = table.getDouble("BigAppliedVolts", bigAppliedVolts);
      bigCurrentAmps = table.getDoubleArray("BigCurrentAmps", bigCurrentAmps);
      bigTempCelcius = table.getDoubleArray("BigTempCelcius", bigTempCelcius);

      littlePositionRad =
          table.getDouble("LittlePositionRad", littlePositionRad);
      littleVelocityRadPerSec =
          table.getDouble("LittleVelocityRadPerSec", littleVelocityRadPerSec);
      littleAppliedVolts =
          table.getDouble("LittleAppliedVolts", littleAppliedVolts);
      littleCurrentAmps =
          table.getDoubleArray("LittleCurrentAmps", littleCurrentAmps);
      littleTempCelcius =
          table.getDoubleArray("LittleTempCelcius", littleTempCelcius);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FlywheelsIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double bigVolts, double littleVolts) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(double bigVelocityRadPerSec,
      double littleVelocityRadPerSec, double bigFFVolts,
      double littleFFVolts) {}

  /** Enable or disable brake mode. */
  public default void setBrakeMode(boolean enable) {}

  /** Set velocity PID constants. */
  public default void configurePID(double bigKp, double bigKi, double bigKd,
      double littleKp, double littleKi, double littleKd) {}
}
