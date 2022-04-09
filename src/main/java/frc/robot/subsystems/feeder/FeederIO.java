// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Feeder subsystem hardware interface. */
public interface FeederIO {
  /** Contains all of the input data received from hardware. */
  public static class FeederIOInputs implements LoggableInputs {
    public boolean lowerProxSensor1 = true;
    public boolean lowerProxSensor2 = true;
    public boolean upperProxSensor1 = true;
    public boolean upperProxSensor2 = true;
    public boolean colorSensorConnected = false;
    public int colorSensorRed = 0;
    public int colorSensorGreen = 0;
    public int colorSensorBlue = 0;
    public int colorSensorProx = 0;

    public double hopperPositionRad = 0.0;
    public double hopperVelocityRadPerSec = 0.0;
    public double hopperAppliedVolts = 0.0;
    public double[] hopperCurrentAmps = new double[] {};
    public double[] hopperTempCelcius = new double[] {};

    public double towerPositionRad = 0.0;
    public double towerVelocityRadPerSec = 0.0;
    public double towerAppliedVolts = 0.0;
    public double[] towerCurrentAmps = new double[] {};
    public double[] towerTempCelcius = new double[] {};

    public double kickerPositionRad = 0.0;
    public double kickerVelocityRadPerSec = 0.0;
    public double kickerAppliedVolts = 0.0;
    public double[] kickerCurrentAmps = new double[] {};
    public double[] kickerTempCelcius = new double[] {};

    public void toLog(LogTable table) {
      table.put("LowerProxSensor1", lowerProxSensor1);
      table.put("LowerProxSensor2", lowerProxSensor2);
      table.put("UpperProxSensor1", upperProxSensor1);
      table.put("UpperProxSensor2", upperProxSensor2);
      table.put("ColorSensorConnected", colorSensorConnected);
      table.put("ColorSensorRed", colorSensorRed);
      table.put("ColorSensorGreen", colorSensorGreen);
      table.put("ColorSensorBlue", colorSensorBlue);
      table.put("ColorSensorProx", colorSensorProx);

      table.put("Hopper/PositionRad", hopperPositionRad);
      table.put("Hopper/VelocityRadPerSec", hopperVelocityRadPerSec);
      table.put("Hopper/AppliedVolts", hopperAppliedVolts);
      table.put("Hopper/CurrentAmps", hopperCurrentAmps);
      table.put("Hopper/TempCelcius", hopperTempCelcius);

      table.put("Tower/PositionRad", towerPositionRad);
      table.put("Tower/VelocityRadPerSec", towerVelocityRadPerSec);
      table.put("Tower/AppliedVolts", towerAppliedVolts);
      table.put("Tower/CurrentAmps", towerCurrentAmps);
      table.put("Tower/TempCelcius", towerTempCelcius);

      table.put("Kicker/PositionRad", kickerPositionRad);
      table.put("Kicker/VelocityRadPerSec", kickerVelocityRadPerSec);
      table.put("Kicker/AppliedVolts", kickerAppliedVolts);
      table.put("Kicker/CurrentAmps", kickerCurrentAmps);
      table.put("Kicker/TempCelcius", kickerTempCelcius);

    }

    public void fromLog(LogTable table) {
      lowerProxSensor1 = table.getBoolean("LowerProxSensor1", lowerProxSensor1);
      lowerProxSensor2 = table.getBoolean("LowerProxSensor2", lowerProxSensor2);
      upperProxSensor1 = table.getBoolean("UpperProxSensor1", upperProxSensor1);
      upperProxSensor2 = table.getBoolean("UpperProxSensor2", upperProxSensor2);
      colorSensorConnected =
          table.getBoolean("ColorSensorConnected", colorSensorConnected);
      colorSensorRed = table.getInteger("ColorSensorRed", colorSensorRed);
      colorSensorGreen = table.getInteger("ColorSensorGreen", colorSensorGreen);
      colorSensorBlue = table.getInteger("ColorSensorBlue", colorSensorBlue);
      colorSensorProx = table.getInteger("ColorSensorProx", colorSensorProx);

      hopperPositionRad =
          table.getDouble("Hopper/PositionRad", hopperPositionRad);
      hopperVelocityRadPerSec =
          table.getDouble("Hopper/VelocityRadPerSec", hopperVelocityRadPerSec);
      hopperAppliedVolts =
          table.getDouble("Hopper/AppliedVolts", hopperAppliedVolts);
      hopperCurrentAmps =
          table.getDoubleArray("Hopper/CurrentAmps", hopperCurrentAmps);
      hopperTempCelcius =
          table.getDoubleArray("Hopper/TempCelcius", hopperTempCelcius);

      towerPositionRad = table.getDouble("Tower/PositionRad", towerPositionRad);
      towerVelocityRadPerSec =
          table.getDouble("Tower/VelocityRadPerSec", towerVelocityRadPerSec);
      towerAppliedVolts =
          table.getDouble("Tower/AppliedVolts", towerAppliedVolts);
      towerCurrentAmps =
          table.getDoubleArray("Tower/CurrentAmps", towerCurrentAmps);
      towerTempCelcius =
          table.getDoubleArray("Tower/TempCelcius", towerTempCelcius);

      kickerPositionRad =
          table.getDouble("Kicker/PositionRad", kickerPositionRad);
      kickerVelocityRadPerSec =
          table.getDouble("Kicker/VelocityRadPerSec", kickerVelocityRadPerSec);
      kickerAppliedVolts =
          table.getDouble("Kicker/AppliedVolts", kickerAppliedVolts);
      kickerCurrentAmps =
          table.getDoubleArray("Kicker/CurrentAmps", kickerCurrentAmps);
      kickerTempCelcius =
          table.getDoubleArray("Kicker/TempCelcius", kickerTempCelcius);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FeederIOInputs inputs) {}

  /** Run open loop at the specified voltage on the hopper. */
  public default void setHopperVoltage(double volts) {}

  /** Enable or disable brake mode on the hopper. */
  public default void setHopperBrakeMode(boolean enable) {}

  /** Run open loop at the specified voltage on the tower. */
  public default void setTowerVoltage(double volts) {}

  /** Enable or disable brake mode on the tower. */
  public default void setTowerBrakeMode(boolean enable) {}

  /** Run open loop at the specified voltage on the kicker. */
  public default void setKickerVoltage(double volts) {}

  /** Enable or disable brake mode on the kicker. */
  public default void setKickerBrakeMode(boolean enable) {}
}
