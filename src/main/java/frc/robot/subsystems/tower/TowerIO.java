// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tower;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Tower subsystem hardware interface. */
public interface TowerIO {
  /** Contains all of the input data received from hardware. */
  public static class TowerIOInputs implements LoggableInputs {
    public boolean cargoSensorsAvailable = false;
    public boolean lowerCargoSensor1 = false;
    public boolean lowerCargoSensor2 = false;
    public boolean upperCargoSensor1 = false;
    public boolean upperCargoSensor2 = false;

    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};

    public void toLog(LogTable table) {
      table.put("CargoSensorAvailable", cargoSensorsAvailable);
      table.put("LowerCargoSensor1", lowerCargoSensor1);
      table.put("LowerCargoSensor2", lowerCargoSensor2);
      table.put("UpperCargoSensor1", upperCargoSensor1);
      table.put("UpperCargoSensor2", upperCargoSensor1);

      table.put("PositionRad", positionRad);
      table.put("VelocityRadPerSec", velocityRadPerSec);
      table.put("AppliedVolts", appliedVolts);
      table.put("CurrentAmps", currentAmps);
      table.put("TempCelcius", tempCelcius);
    }

    public void fromLog(LogTable table) {
      cargoSensorsAvailable =
          table.getBoolean("CargoSensorsAvailable", cargoSensorsAvailable);
      lowerCargoSensor1 =
          table.getBoolean("LowerCargoSensor1", lowerCargoSensor1);
      lowerCargoSensor2 =
          table.getBoolean("LowerCargoSensor1", lowerCargoSensor2);
      upperCargoSensor1 =
          table.getBoolean("UpperCargoSensor1", upperCargoSensor1);
      upperCargoSensor2 =
          table.getBoolean("UpperCargoSensor2", upperCargoSensor2);

      positionRad = table.getDouble("PositionRad", positionRad);
      velocityRadPerSec =
          table.getDouble("VelocityRadPerSec", velocityRadPerSec);
      appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
      currentAmps = table.getDoubleArray("CurrentAmps", currentAmps);
      tempCelcius = table.getDoubleArray("TempCelcius", tempCelcius);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(TowerIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Enable or disable brake mode. */
  public default void setBrakeMode(boolean enable) {}
}
