// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Vision subsystem hardware interface. */
public interface VisionIO {
  /** The set of loggable inputs for the vision subsystem. */
  public static class VisionIOInputs implements LoggableInputs {
    public double captureTimestamp = 0.0;
    public double[] cornerX = new double[] {};
    public double[] cornerY = new double[] {};
    public boolean simpleValid = false;
    public double simpleAngle = 0.0;

    public void toLog(LogTable table) {
      table.put("CaptureTimestamp", captureTimestamp);
      table.put("CornerX", cornerX);
      table.put("CornerY", cornerY);
      table.put("SimpleValid", simpleValid);
      table.put("SimpleAngle", simpleAngle);
    }

    public void fromLog(LogTable table) {
      captureTimestamp = table.getDouble("CaptureTimestamp", captureTimestamp);
      cornerX = table.getDoubleArray("CornerX", cornerX);
      cornerY = table.getDoubleArray("CornerY", cornerY);
      simpleValid = table.getBoolean("SimpleValid", simpleValid);
      simpleAngle = table.getDouble("SimpleAngle", simpleAngle);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}

  /** Enabled or disabled vision LEDs. */
  public default void setLeds(boolean enabled) {}

}
