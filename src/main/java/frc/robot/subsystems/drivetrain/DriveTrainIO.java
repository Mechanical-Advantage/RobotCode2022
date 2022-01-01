// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Drive subsystem hardware interface. */
public interface DriveTrainIO {
  /** The set of loggable inputs for the drive subsystem. */
  public static class DriveTrainIOInputs implements LoggableInputs {
    public double leftPositionRad = 0.0;
    public double leftVelocityRadPerSec = 0.0;
    public double leftAppliedVolts = 0.0;
    public double[] leftCurrentAmps = new double[] {};
    public double[] leftTempCelcius = new double[] {};

    public double rightPositionRad = 0.0;
    public double rightVelocityRadPerSec = 0.0;
    public double rightAppliedVolts = 0.0;
    public double[] rightCurrentAmps = new double[] {};
    public double[] rightTempCelcius = new double[] {};

    public double gyroPositionRad = 0.0;
    public double gyroVelocityRadPerSec = 0.0;

    public void toLog(LogTable table) {
      table.put("LeftPositionRad", leftPositionRad);
      table.put("LeftVelocityRadPerSec", leftVelocityRadPerSec);
      table.put("LeftAppliedVolts", leftAppliedVolts);
      table.put("LeftCurrentAmps", leftCurrentAmps);
      table.put("LeftTempCelcius", leftTempCelcius);

      table.put("RightPositionRad", rightPositionRad);
      table.put("RightVelocityRadPerSec", rightVelocityRadPerSec);
      table.put("RightAppliedVolts", rightAppliedVolts);
      table.put("RightCurrentAmps", rightCurrentAmps);
      table.put("RightTempCelcius", rightTempCelcius);

      table.put("GyroPositionRad", gyroPositionRad);
      table.put("GyroVelocityRadPerSec", gyroVelocityRadPerSec);
    }

    public void fromLog(LogTable table) {
      leftPositionRad = table.getDouble("LeftPositionRad", leftPositionRad);
      leftVelocityRadPerSec = table.getDouble("LeftVelocityRadPerSec", leftVelocityRadPerSec);
      leftAppliedVolts = table.getDouble("LeftAppliedVolts", leftAppliedVolts);
      leftCurrentAmps = table.getDoubleArray("LeftCurrentAmps", leftCurrentAmps);
      leftTempCelcius = table.getDoubleArray("LeftTempCelcius", leftTempCelcius);

      rightPositionRad = table.getDouble("RightPositionRad", rightPositionRad);
      rightVelocityRadPerSec = table.getDouble("RightVelocityRadPerSec", rightVelocityRadPerSec);
      rightAppliedVolts = table.getDouble("RightAppliedVolts", rightAppliedVolts);
      rightCurrentAmps = table.getDoubleArray("RightCurrentAmps", rightCurrentAmps);
      rightTempCelcius = table.getDoubleArray("RightTempCelcius", rightTempCelcius);

      gyroPositionRad = table.getDouble("GyroPositionRad", gyroPositionRad);
      gyroVelocityRadPerSec = table.getDouble("GyroVelocityRadPerSec", gyroVelocityRadPerSec);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(DriveTrainIOInputs inputs) {
  }

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double leftVolts, double rightVolts) {
  }

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(double leftVelocityRadPerSec, double rightVelocityRadPerSec, double leftFFVolts,
      double rightFFVolts) {
  }

  /** Enable or disable brake mode. */
  public default void setBrakeMode(boolean enable) {
  }

  /** Set velocity PID constants. */
  public default void configurePID(double kp, double ki, double kd) {
  }

  /** Reset the encoder(s) to a known position. */
  public default void resetPosition(double leftPositionRad, double rightPositionRad) {
  }
}
